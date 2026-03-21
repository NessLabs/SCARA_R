#ifndef _SIMPLEBDC_H
#define _SIMPLEBDC_H
 
#include "driver/gpio.h"
#include "SimplePWM.h"
#include "QuadratureEncoder.h"
#include "SimplePID.h"
 
class SimpleBDC
{
public:
    SimpleBDC();
 
    // pwm_pin     : PWM output pin
    // dir_pin     : direction GPIO
    // enc_pins    : {pinA, pinB} for quadrature encoder
    // gains       : {Kp, Ki, Kd}
    // dt_s        : control loop period in seconds
    // deg_per_edge: 360.0 / (CPR * 4 * gear_ratio)
    // timer_cfg   : pointer to a TimerConfig (one per unique timer/frequency)
    // channel     : unique LEDC channel per motor instance
    void setup(uint8_t pwm_pin, uint8_t dir_pin,
               uint8_t enc_pins[2], float gains[3],
               float dt_s, float deg_per_edge,
               TimerConfig* timer_cfg, uint8_t channel);
 
    void setTarget(float angleDeg);  // set desired joint angle
    void update();                    // call at fixed rate from timer task
    void stop();                      // zero output, keep state
    void reset();                     // stop + clear PID + zero encoder
 
    float getAngle();                 // current angle from encoder (deg)
    float getSpeed();                 // current speed from encoder (deg/s)
    float getTarget();                // current target angle (deg)
 
private:
    void writeMotor(float output);    // output: -1.0 to 1.0
 
    SimplePWM            _pwm;
    QuadratureEncoder    _encoder;
    SimplePID            _pid;
 
    gpio_num_t           _dir_pin;
 
    float _target  = 0.0f;
    bool  _running = false;
};
 
#endif // _SIMPLEBDC_H