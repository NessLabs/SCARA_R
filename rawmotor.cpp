#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "SimplePWM.h"

static const char* TAG = "RawMotorTest";

#define BDC_PWM_PIN  15
#define BDC_IN1_PIN  22
#define BDC_IN2_PIN  23

SimplePWM pwm;

extern "C" void app_main()
{
    // Setup direction pins
    gpio_config_t io = {};
    io.mode         = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = (1ULL << BDC_IN1_PIN) | (1ULL << BDC_IN2_PIN);
    io.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io);

    // Setup PWM
    TimerConfig timer;
    timer.frequency = 20000;
    pwm.setup(BDC_PWM_PIN, 0, &timer);

    ESP_LOGI(TAG, "Forward 50%% for 2s");
    gpio_set_level((gpio_num_t)BDC_IN1_PIN, 1);
    gpio_set_level((gpio_num_t)BDC_IN2_PIN, 0);
    pwm.setDuty(50.0f);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Stop for 1s");
    gpio_set_level((gpio_num_t)BDC_IN1_PIN, 0);
    gpio_set_level((gpio_num_t)BDC_IN2_PIN, 0);
    pwm.setDuty(0.0f);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Reverse 50%% for 2s");
    gpio_set_level((gpio_num_t)BDC_IN1_PIN, 0);
    gpio_set_level((gpio_num_t)BDC_IN2_PIN, 1);
    pwm.setDuty(50.0f);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Stop");
    pwm.setDuty(0.0f);
    gpio_set_level((gpio_num_t)BDC_IN1_PIN, 0);
    gpio_set_level((gpio_num_t)BDC_IN2_PIN, 0);
}