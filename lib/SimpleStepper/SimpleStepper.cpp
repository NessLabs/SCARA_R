#include "SimpleStepper.h"
#include "rom/ets_sys.h"  // ets_delay_us

SimpleStepper::SimpleStepper()
{
}

void SimpleStepper::setup(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin,
                           int steps_per_rev, int microsteps, float gear_ratio)
{
    _step_pin      = (gpio_num_t)step_pin;
    _dir_pin       = (gpio_num_t)dir_pin;
    _en_pin        = (gpio_num_t)en_pin;
    _steps_per_rev = steps_per_rev;
    _microsteps    = microsteps;
    _gear_ratio    = gear_ratio;

    gpio_config_t io_conf = {};
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << step_pin) | (1ULL << dir_pin) | (1ULL << en_pin);
    gpio_config(&io_conf);

    disable();  // start disabled
}

void SimpleStepper::enable()  { gpio_set_level(_en_pin, 0); }  // active low
void SimpleStepper::disable() { gpio_set_level(_en_pin, 1); }

void SimpleStepper::setTarget(float angleDeg)
{
    _target_steps = degreesToSteps(angleDeg);
    if (_current_steps != _target_steps)
        enable();  // wake up motor only if there's somewhere to go
}

void SimpleStepper::update()
{
    if (_current_steps == _target_steps)
    {
        disable();  // target reached — cut power to avoid heating
        return;
    }

    // Set direction
    if (_target_steps > _current_steps)
    {
        gpio_set_level(_dir_pin, 1);
        _current_steps = _current_steps + 1;
    }
    else
    {
        gpio_set_level(_dir_pin, 0);
        _current_steps = _current_steps - 1;
    }

    // Pulse step pin
    gpio_set_level(_step_pin, 1);
    ets_delay_us(5);
    gpio_set_level(_step_pin, 0);
}

float SimpleStepper::getAngle() { return stepsToDegrees(_current_steps); }
long  SimpleStepper::getSteps() { return _current_steps; }
bool  SimpleStepper::atTarget() { return _current_steps == _target_steps; }

void SimpleStepper::reset()
{
    _current_steps = 0;
    _target_steps  = 0;
}

// ─── private ────────────────────────────────────────────────────────

long SimpleStepper::degreesToSteps(float deg)
{
    // steps = (deg / 360) * steps_per_rev * microsteps * gear_ratio
    return (long)((deg / 360.0f) * _steps_per_rev * _microsteps * _gear_ratio);
}

float SimpleStepper::stepsToDegrees(long steps)
{
    return (steps * 360.0f) / (_steps_per_rev * _microsteps * _gear_ratio);
}