#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_attr.h>
#include <SimplePWM.h>

class Ultrasonic
{
public:
    Ultrasonic();
    ~Ultrasonic();
    void setup(uint8_t gpio_echo, uint8_t gpio_trig, uint8_t ch_trig, TimerConfig timer_config);
    float getDistance();

private:
    SimplePWM trigger;
    void IRAM_ATTR handler();
    gpio_num_t _gpio_echo;
    volatile int64_t _prev_micros, _echo_time;
    volatile uint8_t _states[2];
};

#endif // __ULTRASONIC_H__