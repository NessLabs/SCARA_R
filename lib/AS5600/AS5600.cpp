#include "AS5600.h"
#include "esp_log.h"

static const char* TAG = "AS5600";

AS5600::AS5600()
{
}

void AS5600::setup(uint8_t sda_pin, uint8_t scl_pin, float gear_ratio)
{
    _gear_ratio = gear_ratio;
    _i2c.setup(AS5600_ADDR, 400000, sda_pin, scl_pin);
}

int AS5600::getRaw()
{
    uint8_t reg[1]  = { AS5600_REG_ANGLE };
    uint8_t data[2] = { 0, 0 };
    _i2c.read(reg, 1, data, 2);
    return ((int)data[0] << 8) | data[1];
}

float AS5600::getAngle()
{
    float angle = (getRaw() / 4096.0f) * 360.0f / _gear_ratio;
    return angle - _offset;
}

uint8_t AS5600::getStatus()
{
    uint8_t reg[1]  = { AS5600_REG_STATUS };
    uint8_t data[1] = { 0 };
    _i2c.read(reg, 1, data, 1);
    return data[0];
}

bool AS5600::isConnected()
{
    // if magnet ok bit is set, sensor is alive and magnet is in range
    uint8_t status = getStatus();
    if (!(status & AS5600_MAGNET_OK))
    {
        ESP_LOGW(TAG, "AS5600 status: 0x%02X — check magnet position", status);
        return false;
    }
    return true;
}

void AS5600::setZero()
{
    _offset = (getRaw() / 4096.0f) * 360.0f / _gear_ratio;
    ESP_LOGI(TAG, "Zero set at offset %.2f deg", _offset);
}