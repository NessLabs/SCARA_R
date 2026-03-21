#ifndef _SIMPLEI2C_H
#define _SIMPLEI2C_H

#include "driver/i2c_master.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_TIMEOUT_MS 10

class SimpleI2C
{
public:
    SimpleI2C();
    void setup(uint8_t device_address, uint32_t freq = 400000, uint8_t sda_pin = 4, uint8_t scl_pin = 5, i2c_port_t i2c_num = I2C_NUM_0);
    void read(uint8_t data_in[], size_t size_in, uint8_t data_out[], size_t size_out);
    void write(uint8_t data_in[], size_t size_in);
    i2c_master_bus_handle_t _bus_handle;

private:
    uint8_t _device_address;
    i2c_master_dev_handle_t _dev_handle;
    static bool _i2c_initialized;
};
#endif // _SIMPLEI2C_H