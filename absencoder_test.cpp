#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "AS5600.h"

static const char* TAG = "AS5600Test";

#define SDA_PIN  21
#define SCL_PIN  22

AS5600 encoder;

float _gear_ratio = 3.0f;

const char* magnetStatus(uint8_t s)
{
    if (s & AS5600_MAGNET_OK)     return "OK";
    if (s & AS5600_MAGNET_WEAK)   return "WEAK (too far)";
    if (s & AS5600_MAGNET_STRONG) return "STRONG (too close)";
    return "NOT DETECTED";
}

extern "C" void app_main()
{
    encoder.setup(SDA_PIN, SCL_PIN);

    if (!encoder.isConnected())
        ESP_LOGW(TAG, "AS5600 not found — check wiring, pullups and magnet");
    else
    {
        encoder.setZero();
        ESP_LOGI(TAG, "AS5600 ready");
    }

    while (true)
    {
        uint8_t status = encoder.getStatus();
        ESP_LOGI(TAG, "angle: %.2f deg | raw: %d | magnet: %s",
                 encoder.getAngle(), encoder.getRaw(), magnetStatus(status));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}