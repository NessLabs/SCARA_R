#ifndef _AS5600_H
#define _AS5600_H

#include "SimpleI2C.h"

#define AS5600_ADDR       0x36
#define AS5600_REG_ANGLE  0x0C  // angle high byte (low byte at 0x0D)
#define AS5600_REG_STATUS 0x0B  // magnet status register

// Status register bits
#define AS5600_MAGNET_OK   0x20  // magnet detected, field strength good
#define AS5600_MAGNET_WEAK 0x10  // magnet too far
#define AS5600_MAGNET_STRONG 0x08 // magnet too close

class AS5600
{
public:
    AS5600();

    void  setup(uint8_t sda_pin, uint8_t scl_pin, float gear_ratio = 1.0f);

    float getAngle();      // joint angle in degrees (accounts for gear ratio)
    int   getRaw();        // raw 12-bit value (0-4095)
    uint8_t getStatus();   // raw status register byte
    bool  isConnected();   // true if sensor responds on I2C
    void  setZero();       // set current position as 0 reference

private:
    SimpleI2C _i2c;
    float     _gear_ratio;
    float     _offset = 0.0f;  // set by setZero()
};

#endif // _AS5600_H
