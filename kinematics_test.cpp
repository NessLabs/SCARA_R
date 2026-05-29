#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Kinematics.h"

static const char* TAG = "KinTest";

void printJointState(const char* label, const JointState& js)
{
    ESP_LOGI(TAG, "%s: t1=%.2f  t2=%.2f  t3=%.2f  d4=%.2f",
             label, js.theta1, js.theta2, js.theta3, js.d4);
}

void printOpState(const char* label, const OpState& op)
{
    ESP_LOGI(TAG, "%s: x=%.2f  y=%.2f  z=%.2f  phi=%.2f",
             label, op.x, op.y, op.z, op.phi);
}

// ─── Test 1: FK ───────────────────────────────────────────────────────
// Give known joint angles → get end-effector position
void testFK()
{
    ESP_LOGI(TAG, "=== FK Test ===");

    JointState js = { 30.0f, 45.0f, 0.0f, 20.0f };
    printJointState("Input", js);

    OpState op = getOpState(js);
    printOpState("FK result", op);
}

// ─── Test 2: IK round-trip ────────────────────────────────────────────
// FK → IK → compare original vs recovered joint angles
void testIKRoundTrip()
{
    ESP_LOGI(TAG, "=== IK Round-trip Test ===");

    JointState original = { 30.0f, 45.0f, 0.0f, 20.0f };
    printJointState("Original joints", original);

    // FK to get op state
    OpState op = getOpState(original);
    printOpState("FK result", op);

    // IK to recover joints
    JointState recovered;
    if (!scaraIK(op, recovered))
    {
        ESP_LOGE(TAG, "IK failed — out of reach");
        return;
    }
    printJointState("IK recovered", recovered);

    // error should be near zero
    ESP_LOGI(TAG, "Error: t1=%.4f  t2=%.4f  t3=%.4f  d4=%.4f",
             original.theta1 - recovered.theta1,
             original.theta2 - recovered.theta2,
             original.theta3 - recovered.theta3,
             original.d4     - recovered.d4);
}

// ─── Test 3: Both IK solutions ────────────────────────────────────────
// Show both elbow-up and elbow-down solutions for the same target
void testBothSolutions()
{
    ESP_LOGI(TAG, "=== Both IK Solutions Test ===");

    OpState target = { 150.0f, 80.0f, 30.0f, 0.0f };
    printOpState("Target", target);

    IKSolutions ik = inverseKinematics(target);
    ESP_LOGI(TAG, "Solutions found: %d", ik.count);

    for (int i = 0; i < ik.count; i++)
    {
        char label[32];
        snprintf(label, sizeof(label), "Solution %d", i + 1);
        printJointState(label, ik.solutions[i]);
    }

    // pick best solution from current position (all zeros)
    if (ik.count > 0)
    {
        JointState current = { 0.0f, 0.0f, 0.0f, 0.0f };
        float weights[3]   = { 1.0f, 1.0f, 1.0f };
        int best = findBestSolution(ik, current, weights);
        ESP_LOGI(TAG, "Best solution from home: %d", best + 1);
        printJointState("Best", ik.solutions[best]);
    }
}

// ─── Test 4: Reachability ─────────────────────────────────────────────
// Test points inside and outside workspace
void testReachability()
{
    ESP_LOGI(TAG, "=== Reachability Test ===");

    // max reach = L1 + L2 = 260mm
    struct { float x; float y; const char* desc; } points[] = {
        { 100.0f,  100.0f, "inside workspace" },
        { 200.0f,    0.0f, "near max reach"   },
        { 300.0f,    0.0f, "out of reach"     },
        {   5.0f,    0.0f, "near min reach"   },
        {   0.0f,    0.0f, "at origin"        },
    };

    for (auto& p : points)
    {
        OpState op = { p.x, p.y, 0.0f, 0.0f };
        IKSolutions ik = inverseKinematics(op);
        ESP_LOGI(TAG, "(%6.1f, %6.1f) %-20s → %d solution(s)",
                 p.x, p.y, p.desc, ik.count);
    }
}

extern "C" void app_main()
{
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "L1=%.1f mm  L2=%.1f mm  max_reach=%.1f mm",
             L1, L2, L1 + L2);

    testFK();
    vTaskDelay(pdMS_TO_TICKS(100));

    testIKRoundTrip();
    vTaskDelay(pdMS_TO_TICKS(100));

    testBothSolutions();
    vTaskDelay(pdMS_TO_TICKS(100));

    testReachability();

    ESP_LOGI(TAG, "=== All tests done ===");

    while (true)
        vTaskDelay(pdMS_TO_TICKS(5000));
}
