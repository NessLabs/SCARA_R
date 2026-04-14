#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "QuadratureEncoder.h"

static const char* TAG = "QuadTest";

// GPIO 34/35 have no internal pullups on ESP32 — use 18/19 instead
#define PIN_A  18
#define PIN_B  19

QuadratureEncoder enc;

extern "C" void app_main()
{
    uint8_t pins[2] = { PIN_A, PIN_B };
    // degrees per edge = 360 / (CPR * 4) — update CPR to match your encoder
    enc.setup(pins, 360.0f / (1026.f * 4));

    while (true)
    {
        ESP_LOGI(TAG, "angle: %.2f deg | speed: %.2f deg/s | dir: %d",
                 enc.getAngle(), enc.getSpeed(), enc.getDirection());
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}