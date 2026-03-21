#include "SimplePWM.h"

SimplePWM::SimplePWM()
{
}

void SimplePWM::setup(const uint8_t pin, const uint8_t channel, TimerConfig *timer_config, bool invert)
{
    static bool fade_installed = false;
    gpio_num = (gpio_num_t)pin;
    _channel = (ledc_channel_t)channel;
    _timer_config = timer_config;
    _max_digital_level = (1 << timer_config->bit_resolution) - 1;
    ledc_timer_config_t ledc_timer = {
        .speed_mode = timer_config->mode,
        .duty_resolution = timer_config->bit_resolution,
        .timer_num = timer_config->timer,
        .freq_hz = timer_config->frequency,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK)
        printf("Timer config failed! Timer=%d, Mode=%d, err=%d\n", timer_config->timer, timer_config->mode, err);
    ledc_channel_config_t ledc_channel = {
        .gpio_num = gpio_num,
        .speed_mode = timer_config->mode,
        .channel = _channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer_config->timer,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = invert,
    };
    _last_level = 0;
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK)
        printf("LED set channel failed! err = %d\n", err);
    if (!fade_installed)
    {
        ledc_fade_func_install(0);
        fade_installed = true;
    }
    printf("PWM Setup: GPIO=%d, Channel=%d, Timer=%d, Mode=%d\n", pin, channel, timer_config->timer, timer_config->mode);
}

void SimplePWM::setDigitalLevel(uint32_t digital_level, uint32_t hpoint)
{
    if (digital_level != _last_level)
    {
        if (digital_level > _max_digital_level)
            digital_level = _max_digital_level;
        esp_err_t err = ledc_set_duty_and_update(_timer_config->mode, _channel, digital_level, hpoint);
        if (err != ESP_OK)
            printf("Failed to set duty: channel=%d, mode=%d, level=%ld, err=%d\n", _channel, _timer_config->mode, digital_level, err);
        _last_level = digital_level;
    }
}
void SimplePWM::setDuty(float duty_percentage)
{
    uint32_t digital_level = (uint32_t)(duty_percentage * _max_digital_level / 100.0f);
    setDigitalLevel(digital_level, 0);
}

void SimplePWM::setFrequency(uint32_t frequency)
{
    esp_err_t err = ledc_set_freq(_timer_config->mode, _timer_config->timer, frequency);
    if (err != ESP_OK)
            printf("Failed to set freq: channel=%d, mode=%d,  timer=%d, err=%d\n", _channel, _timer_config->mode,_timer_config->timer, err);
}