#if !defined(_PCF8575_H)
#define _PCF8575_H

#include <stdint.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>

class Pcf8575
{
private:
  gpio_num_t _sda_pin;
  gpio_num_t _scl_pin;
  gpio_num_t _intr_pin;
  gpio_isr_t _isr_handler;

  i2c_master_bus_config_t _bus_cfg;
  i2c_master_bus_handle_t _bus_handle;
  i2c_device_config_t _dev_cfg;
  i2c_master_dev_handle_t _dev_handle;

  bool _exclusive_bus;

  uint16_t _last_state;

public:
  struct Address
  {
    bool a0;
    bool a1;
    bool a2;
  };

  enum Pin
  {
    P00 = 0,
    P01,
    P02,
    P03,
    P04,
    P05,
    P06,
    P07,
    P10,
    P11,
    P12,
    P13,
    P14,
    P15,
    P16,
    P17,
  };

  enum Level
  {
    Low = 0,
    High = 1
  };

  static const uint8_t BASE_ADDRESS_7 = 0b0100000;
  static const uint32_t MAX_RATE = 400000; // 100 kHz

  static const i2c_master_bus_config_t DEFAULT_BUS_CONFIG;

  static const size_t PAYLOAD_SIZE = 2;
  static const uint8_t MAX_PIN_NUM = 8 * PAYLOAD_SIZE - 1;

  static bool getBit(uint8_t data[PAYLOAD_SIZE], Pin pin);
  static void setBit(uint8_t data[PAYLOAD_SIZE], Pin pin, bool value);

  // Creates new I2C master bus
  Pcf8575(gpio_num_t sda_pin, gpio_num_t scl_pin, Address addr, gpio_num_t intr_pin = GPIO_NUM_NC, i2c_master_bus_config_t bus_cfg = DEFAULT_BUS_CONFIG);
  // Use existing I2C master bus
  Pcf8575(i2c_master_bus_handle_t handle, Address addr, gpio_num_t intr_pin = GPIO_NUM_NC);
  ~Pcf8575();

  bool registerInterrupt(gpio_isr_t handler, void *arg);
  void deregisterInterrupt();

  void write(const uint8_t *data, int timeout = -1);
  void write(Pin pin, Level level, int timeout = -1);
  void read(uint8_t *data, int timeout = -1);
  int read(Pin pin, int timeout = -1);

  // Get last read values
  uint16_t readCached();
  int readCached(Pin pin);
};

#endif // _PCF8574_H
