#ifndef __SIMPLETIMER_H__
#define __SIMPLETIMER_H__

#include <esp_timer.h>
#include <esp_task_wdt.h>

class SimpleTimer
{
public:
    SimpleTimer();
    void setup(esp_timer_cb_t interrupt, const char name[]);
    void startPeriodic(uint64_t us);
    void stopPeriodic();
    void startOnce(uint64_t us);
    void setInterrupt();
    bool interruptAvailable();

private:
    esp_timer_handle_t _timer_handle;
    volatile bool _interrupt_flag = false;
};

#endif // __SIMPLETIMER_H__