#ifndef _SIMPLEBDC_H
#define _SIMPLEBDC_H

#include "driver/gpio.h"
#include "SimplePWM.h"
#include "QuadratureEncoder.h"
#include "SimplePID.h"

#define BDC_TOLERANCE 2.0f  // degrees — tune as needed

class SimpleBDC
{
public:
    SimpleBDC();

    void setup(uint8_t pwm_pin, uint8_t in1_pin, uint8_t in2_pin,
               uint8_t enc_pins[2], float gains[3],
               float dt_s, float deg_per_edge,
               TimerConfig* timer_cfg, uint8_t channel);

    void setTarget(float angleDeg);
    void update();
    void stop();
    void reset();

    float getAngle();
    float getSpeed();
    float getTarget();
    bool  atTarget();  // true when within BDC_TOLERANCE

private:
    void writeMotor(float output);

    SimplePWM         _pwm;
    QuadratureEncoder _encoder;
    SimplePID         _pid;

    gpio_num_t        _in1_pin;
    gpio_num_t        _in2_pin;

    float _target  = 0.0f;
    bool  _running = false;
};

#endif