#ifndef _SIMPLESTEPPER_H
#define _SIMPLESTEPPER_H

#include "driver/gpio.h"

class SimpleStepper
{
public:
    SimpleStepper();

    // step_pin, dir_pin, en_pin: GPIO pins
    // steps_per_rev: full steps per revolution (200 for NEMA17)
    // microsteps   : microstepping factor (1, 2, 4, 8, 16...)
    // gear_ratio   : driven/driver pulley teeth ratio
    void setup(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin,
               int steps_per_rev, int microsteps, float gear_ratio);

    void enable();
    void disable();

    // Set desired position in degrees — automatically enables motor
    void setTarget(float angleDeg);

    // Pulse one step toward target — call at fixed rate from timer
    void update();

    float getAngle();  // current position (deg)
    long  getSteps();  // current step count
    bool  atTarget();  // true when position reached

    void reset();      // zero the step counter

private:
    long  degreesToSteps(float deg);
    float stepsToDegrees(long steps);

    gpio_num_t _step_pin, _dir_pin, _en_pin;

    int   _steps_per_rev = 200;
    int   _microsteps    = 1;
    float _gear_ratio    = 1.0f;

    volatile long _current_steps = 0;
    long          _target_steps  = 0;
};

#endif // _SIMPLESTEPPER_H