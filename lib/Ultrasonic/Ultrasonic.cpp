#include "Ultrasonic.h"
#include <esp_log.h>

Ultrasonic::Ultrasonic()
{
}

Ultrasonic::~Ultrasonic()
{
    gpio_isr_handler_remove(_gpio_echo);
}

void Ultrasonic::setup(uint8_t gpio_echo, uint8_t gpio_trig, uint8_t ch_trig, TimerConfig timer_config)
{
    trigger.setup(gpio_trig, ch_trig, timer_config, 0);
    trigger.setDuty(0.1f);
    _gpio_echo = (gpio_num_t)gpio_echo;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pin_bit_mask = (1ULL << _gpio_echo);
    gpio_config(&io_conf);
    gpio_uninstall_isr_service();
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(_gpio_echo, [](void *arg)
                         { static_cast<Ultrasonic *>(arg)->handler(); }, this);
}

float Ultrasonic::getDistance() // mm/us
{
    return 0.343f * _echo_time / 2;
}

void Ultrasonic::handler()
{

    _states[0] = gpio_get_level(_gpio_echo);
    if (_states[0] > _states[1])
        _prev_micros = esp_timer_get_time();
    else
        _echo_time = esp_timer_get_time() - _prev_micros;
    _states[1] = _states[0];
}