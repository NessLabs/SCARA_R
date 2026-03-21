
#include "SimpleBDC.h"
#include "esp_log.h"

static const char* BDC_TAG = "SimpleBDC";

SimpleBDC::SimpleBDC()
{
}

void SimpleBDC::setup(uint8_t pwm_pin, uint8_t dir_pin,
                      uint8_t enc_pins[2], float gains[3],
                      float dt_s, float deg_per_edge,
                      TimerConfig* timer_cfg, uint8_t channel)
{
    _dir_pin = (gpio_num_t)dir_pin;

    // Direction pin
    gpio_config_t io_conf = {};
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << dir_pin);
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // PWM
    _pwm.setup(pwm_pin, channel, timer_cfg);

    // Encoder
    _encoder.setup(enc_pins, deg_per_edge);

    // PID — output -1.0 to 1.0, scaled to PWM duty in writeMotor()
    _pid.setup(gains, dt_s, -1.0f, 1.0f);

    ESP_LOGI(BDC_TAG, "Ready on PWM pin %d, DIR pin %d", pwm_pin, dir_pin);
}

void SimpleBDC::setTarget(float angleDeg)
{
    _target  = angleDeg;
    _running = true;
}

void SimpleBDC::update()
{
    if (!_running) return;
    float error  = _target - _encoder.getAngle();
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

float SimpleBDC::getAngle()  { return _encoder.getAngle(); }
float SimpleBDC::getSpeed()  { return _encoder.getSpeed(); }
float SimpleBDC::getTarget() { return _target; }

// output: -1.0 (full reverse) to +1.0 (full forward)
void SimpleBDC::writeMotor(float output)
{
    if (output >= 0.0f)
        gpio_set_level(_dir_pin, 1);
    else
    {
        gpio_set_level(_dir_pin, 0);
        output = -output;
    }
    _pwm.setDuty(output * 100.0f);  // SimplePWM expects 0-100%
}


// Use example

// TimerConfig bdcTimer;  // shared timer config for both motors
// bdcTimer.frequency = 20000;

// SimpleBDC motorJ2, motorJ3;
// motorJ2.setup(pwmPin2, dirPin2, encPins2, gains2, 0.01f, degPerEdge2, &bdcTimer, 0);
// motorJ3.setup(pwmPin3, dirPin3, encPins3, gains3, 0.01f, degPerEdge3, &bdcTimer, 1);