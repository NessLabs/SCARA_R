#include "SimpleTimer.h"

SimpleTimer::SimpleTimer()
{
}

void SimpleTimer::setup(esp_timer_cb_t interrupt, const char name[])
{
    esp_timer_create_args_t conf;
    conf.callback = interrupt;
    conf.dispatch_method = ESP_TIMER_TASK;
    conf.name = name;
    conf.skip_unhandled_events = true;
    esp_timer_create(&conf, &_timer_handle);
    esp_timer_init();
}

void SimpleTimer::startPeriodic(uint64_t us)
{
    esp_timer_stop(_timer_handle);
    esp_timer_start_periodic(_timer_handle, us);
}

void SimpleTimer::stopPeriodic()
{
    esp_timer_stop(_timer_handle);
}

void SimpleTimer::startOnce(uint64_t us)
{
    esp_timer_stop(_timer_handle);
    esp_timer_start_once(_timer_handle, us);
}

void SimpleTimer::setInterrupt()
{
    _interrupt_flag = true;
}

bool SimpleTimer::interruptAvailable()
{ 
    bool flag = _interrupt_flag;
    _interrupt_flag = false;
    return flag;
}
