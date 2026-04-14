#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "QuadratureEncoder.h"

static const char* TAG = "CPRTest";

#define PIN_A  18
#define PIN_B  19

QuadratureEncoder enc;

extern "C" void app_main()
{
    uint8_t pins[2] = { PIN_A, PIN_B };
    enc.setup(pins, 1.0f);  // 1 deg/edge = raw edge count

    // Let it stabilize
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Reset to zero
    enc.setAngle(0.0f);
    ESP_LOGI(TAG, "Starting — run motor at FULL speed for exactly 5 seconds then stop");
    vTaskDelay(pdMS_TO_TICKS(5000));

    float edges     = enc.getAngle();
    float rpm       = (edges / 5.0f) / 360.0f * 60.0f; // if edges were degrees
    ESP_LOGI(TAG, "Total edges in 5s: %.0f", edges);
    ESP_LOGI(TAG, "That means %.0f edges/rev if motor ran at 130 RPM", edges / (130.0f * 5.0f / 60.0f));
    ESP_LOGI(TAG, "Suggested: enc.setup(pins, 360.0f / %.0f)", edges / (130.0f * 5.0f / 60.0f));
}