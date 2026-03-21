#include "SimpleI2C.h"

bool SimpleI2C::_i2c_initialized = false;

SimpleI2C::SimpleI2C()
{
}

void SimpleI2C::setup(uint8_t device_address, uint32_t freq, uint8_t sda_pin, uint8_t scl_pin, i2c_port_t i2c_num)
{
    if (!_i2c_initialized)
    {
        i2c_master_bus_config_t conf;
        conf.clk_source = I2C_CLK_SRC_RC_FAST;
        conf.scl_io_num = (gpio_num_t)scl_pin;
        conf.sda_io_num = (gpio_num_t)sda_pin;
        conf.i2c_port = i2c_num;
        conf.glitch_ignore_cnt = 7;
        conf.flags.enable_internal_pullup = false;
        conf.intr_priority = 0;
        ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &_bus_handle));
        _i2c_initialized = true;
        ESP_LOGI("I2C ", "Master bus initialized\n");
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = freq,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(_bus_handle, &dev_cfg, &_dev_handle));
    ESP_LOGI("I2C ", "Device configured\n");
}

void SimpleI2C::read(uint8_t data_in[], size_t size_in, uint8_t data_out[], size_t size_out)
{
    esp_err_t ret;
    ESP_ERROR_CHECK(ret = i2c_master_transmit_receive(_dev_handle, data_in, size_in, data_out, size_out, -1));
    if (ret != ESP_OK)
        ESP_LOGE("I2C ", "Read error %s", esp_err_to_name(ret));
}

void SimpleI2C::write(uint8_t data_in[], size_t size_in)
{
    esp_err_t ret;
    ESP_ERROR_CHECK(ret = i2c_master_transmit(_dev_handle, data_in, size_in, -1));
    if (ret != ESP_OK)
        ESP_LOGE("I2C ", "Write error %s", esp_err_to_name(ret));
}
