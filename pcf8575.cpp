#include <pcf8575.h>
#include <driver/gpio.h>
#include <esp_intr_alloc.h>

// Update payload and replace level bit at pin position
void Pcf8575::setBit(uint8_t *data, Pcf8575::Pin pin, bool level)
{
  if (level != 0 && level != 1)
  {
    return;
  }
  uint8_t byte_of_pin = pin / 8;
  uint8_t bit_of_byte = pin % 8;
  data[byte_of_pin] =      // Byte of pin
      (data[byte_of_pin] & // Masked by
       (0b11111111 ^       // Complement of
        (1
         << (bit_of_byte) // Pin position
         ))) |            // Joined with
      (
          level           // level at
          << (bit_of_byte // pin position
              ));
}

const i2c_master_bus_config_t Pcf8575::DEFAULT_BUS_CONFIG = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_NC,
    .scl_io_num = GPIO_NUM_NC,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags = {
        .enable_internal_pullup = true,
    }};

uint16_t mkAddress(Pcf8575::Address addr)
{
  return Pcf8575::BASE_ADDRESS_7 | (addr.a2 << 2) | (addr.a1 << 1) | addr.a0;
}

i2c_device_config_t mkDeviceConfig(Pcf8575::Address addr)
{
  uint16_t address = mkAddress(addr);
  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = address,
      .scl_speed_hz = Pcf8575::MAX_RATE,
  };
  return dev_cfg;
}

bool Pcf8575::getBit(uint8_t data[PAYLOAD_SIZE], Pin pin)
{
  uint8_t byte = data[pin / 8];
  return (byte >> (pin % 8)) & 1;
}

Pcf8575::Pcf8575(gpio_num_t sda_pin, gpio_num_t scl_pin, Pcf8575::Address addr, gpio_num_t intr_pin, i2c_master_bus_config_t bus_cfg)
{
  bus_cfg.sda_io_num = sda_pin;
  bus_cfg.scl_io_num = scl_pin;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &_bus_handle));

  _sda_pin = sda_pin;
  _scl_pin = scl_pin;
  _bus_cfg = bus_cfg;
  _exclusive_bus = true;

  Pcf8575(_bus_handle, addr, intr_pin);
}

Pcf8575::Pcf8575(i2c_master_bus_handle_t handle, Address addr, gpio_num_t intr_pin)
{
  _isr_handler = nullptr;
  _bus_handle = handle;
  _dev_cfg = mkDeviceConfig(addr);
  ESP_ERROR_CHECK(i2c_master_bus_add_device(_bus_handle, &_dev_cfg, &_dev_handle));

  // _exclusive_bus = false;

  if (GPIO_IS_VALID_GPIO(intr_pin))
  {
    _intr_pin = intr_pin;
    ESP_ERROR_CHECK(gpio_reset_pin(intr_pin));
    ESP_ERROR_CHECK(gpio_set_direction(intr_pin, GPIO_MODE_INPUT));
    // ESP_ERROR_CHECK(gpio_set_pull_mode(intr_pin, GPIO_
    ESP_ERROR_CHECK(gpio_pullup_en(intr_pin));
    ESP_ERROR_CHECK(gpio_pulldown_dis(intr_pin));
    ESP_ERROR_CHECK(gpio_set_intr_type(intr_pin, GPIO_INTR_NEGEDGE));
  }
  else
  {
    printf("Invalid Interrupt GPIO pin!");
  }
};

Pcf8575::~Pcf8575()
{
  ESP_ERROR_CHECK(i2c_master_bus_rm_device(_dev_handle));

  if (_exclusive_bus)
  {
    ESP_ERROR_CHECK(i2c_del_master_bus(_bus_handle));
  }

  if (GPIO_IS_VALID_GPIO(_intr_pin))
  {
    ESP_ERROR_CHECK(gpio_reset_pin(_intr_pin));

    if (_isr_handler != nullptr)
    {
      ESP_ERROR_CHECK(gpio_isr_handler_remove(_intr_pin));
    }
  }
}

// Requires gpio_install_isr_service with at least ESP_INTR_FLAG_EDGE
bool Pcf8575::registerInterrupt(gpio_isr_t handler, void *arg)
{
  if (!GPIO_IS_VALID_GPIO(_intr_pin))
  {
    return false;
  }

  _isr_handler = handler;
  // ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM));
  ESP_ERROR_CHECK(gpio_isr_handler_add(_intr_pin, handler, arg));
  ESP_ERROR_CHECK(gpio_intr_enable(_intr_pin));

  return true;
}

void Pcf8575::deregisterInterrupt()
{
  ESP_ERROR_CHECK(gpio_isr_handler_remove(_intr_pin));
}

void Pcf8575::write(const uint8_t *data, int timeout)
{
  ESP_ERROR_CHECK(i2c_master_transmit(_dev_handle, data, 2, timeout));
  _last_state = (data[1] << 8) | data[0];
}

void Pcf8575::write(Pin pin, Level level, int timeout)
{
  uint8_t data[2];
  ESP_ERROR_CHECK(i2c_master_receive(_dev_handle, data, 2, timeout));

  setBit(data, pin, level);

  write(data, timeout);
}

void Pcf8575::read(uint8_t *data, int timeout)
{
  ESP_ERROR_CHECK(i2c_master_receive(_dev_handle, data, 2, timeout));
  _last_state = (data[1] << 8) | data[0];
}

int Pcf8575::read(Pin pin, int timeout)
{
  uint8_t data[2];
  read(data, timeout);
  return (data[pin / 8] >> (pin % 8)) & 1;
}

uint16_t Pcf8575::readCached()
{
  return _last_state;
}

int Pcf8575::readCached(Pin pin)
{
  return (_last_state >> pin) & 1;
}
