#include <stdio.h>
#include "esp_log.h"
#include "KinematicsMath.h"

static const char* TAG = "KMTest";

void testKinematicsMath()
{
    // rz = rotateAxis('z', 180)
    // ry = rotateAxis('y', 60)
    // rx = rotateAxis('x', 15)
    Mat3 rz = rotateAxis('z', 180.0f);
    Mat3 ry = rotateAxis('y', 60.0f);
    Mat3 rx = rotateAxis('x', 15.0f);

    // rt = rz * ry * rx
    Mat3 rt = mat3Mul(mat3Mul(rz, ry), rx);

    // p = [0, -1, 0]
    float p[3] = { 0.0f, -1.0f, 0.0f };

    // pRotated = rt * p
    float pRotated[3] = { 0, 0, 0 };
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 3; k++)
            pRotated[i] += rt.m[i][k] * p[k];

    // Print rt
    ESP_LOGI(TAG, "--- rt = rz * ry * rx ---");
    for (int i = 0; i < 3; i++)
        ESP_LOGI(TAG, "[ %7.4f  %7.4f  %7.4f ]",
                 rt.m[i][0], rt.m[i][1], rt.m[i][2]);

    // Print p
    ESP_LOGI(TAG, "--- p ---");
    for (int i = 0; i < 3; i++)
        ESP_LOGI(TAG, "[ %7.4f ]", p[i]);

    // Print pRotated
    ESP_LOGI(TAG, "--- pRotated = rt * p ---");
    for (int i = 0; i < 3; i++)
        ESP_LOGI(TAG, "[ %7.4f ]", pRotated[i]);
}

extern "C" void app_main()
{
    testKinematicsMath();
}