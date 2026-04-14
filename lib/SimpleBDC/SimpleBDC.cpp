#include "SimpleBDC.h"
#include "esp_log.h"
#include <math.h>

static const char* BDC_TAG = "SimpleBDC";

SimpleBDC::SimpleBDC() {}

void SimpleBDC::setup(uint8_t pwm_pin, uint8_t in1_pin, uint8_t in2_pin,
                      uint8_t enc_pins[2], float gains[3],
                      float dt_s, float deg_per_edge,
                      TimerConfig* timer_cfg, uint8_t channel)
{
    _in1_pin = (gpio_num_t)in1_pin;
    _in2_pin = (gpio_num_t)in2_pin;

    gpio_config_t io_conf = {};
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << _in1_pin) | (1ULL << _in2_pin);
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    _pwm.setup(pwm_pin, channel, timer_cfg);
    _encoder.setup(enc_pins, deg_per_edge);
    _pid.setup(gains, dt_s, -1.0f, 1.0f);

    ESP_LOGI(BDC_TAG, "Ready: PWM %d, IN1 %d, IN2 %d", pwm_pin, in1_pin, in2_pin);
}

void SimpleBDC::setTarget(float angleDeg)
{
    _target  = angleDeg;
    _running = true;
    _pid.reset();  // clear stale integral when new target is set
}

void SimpleBDC::update()
{
    if (!_running) return;

    float error = _target - _encoder.getAngle();

    // within tolerance — stop and hold
    if (fabsf(error) <= BDC_TOLERANCE)
    {
        writeMotor(0.0f);
        return;
    }

    float output = _pid.calc(error);
    writeMotor(output);
}

void SimpleBDC::stop()
{
    _running = false;
    writeMotor(0.0f);
}

void SimpleBDC::reset()
{
    stop();
    _pid.reset();
    _encoder.setAngle(0.0f);
    _target = 0.0f;
}

bool SimpleBDC::atTarget()
{
    return fabsf(_target - _encoder.getAngle()) <= BDC_TOLERANCE;
}

float SimpleBDC::getAngle()  { return _encoder.getAngle(); }
float SimpleBDC::getSpeed()  { return _encoder.getSpeed(); }
float SimpleBDC::getTarget() { return _target; }

void SimpleBDC::writeMotor(float output)
{
    if (output > 0.02f)
    {
        gpio_set_level(_in1_pin, 1);
        gpio_set_level(_in2_pin, 0);
    }
    else if (output < -0.02f)
    {
        gpio_set_level(_in1_pin, 0);
        gpio_set_level(_in2_pin, 1);
        output = -output;
    }
    else
    {
        gpio_set_level(_in1_pin, 0);
        gpio_set_level(_in2_pin, 0);
        output = 0.0f;
    }

    _pwm.setDuty(output * 100.0f);
}