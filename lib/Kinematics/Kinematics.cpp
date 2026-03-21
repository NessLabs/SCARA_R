#include "Kinematics.h"
#include "esp_log.h"
#include <math.h>

static const char* KIN_TAG = "Kinematics";

// ─── Forward Kinematics ───────────────────────────────────────────────
// Origin = center of base body
// J1 pivot is offset BASE_OFFSET mm along X

Mat4 scaraFK(const JointState& js)
{
    // Base offset: translate from robot origin to J1 pivot
    Mat4 T_base      = mat4Identity();
    T_base.m[0][3]   = BASE_OFFSET;

    // DH table: {theta, d, r, alpha}
    DHParams dh1 = { js.theta1, 0,      L1, 0 };
    DHParams dh2 = { js.theta2, 0,      L2, 0 };
    DHParams dh3 = { 0,         js.d3,  0,  0 };  // prismatic Z
    DHParams dh4 = { js.theta4, 0,      0,  0 };  // end-effector rotation

    Mat4 T1 = getHM(dh1);
    Mat4 T2 = getHM(dh2);
    Mat4 T3 = getHM(dh3);
    Mat4 T4 = getHM(dh4);

    // T = T_base * T1 * T2 * T3 * T4
    return mat4Mul(mat4Mul(mat4Mul(mat4Mul(T_base, T1), T2), T3), T4);
}

OpState getOpState(const JointState& js)
{
    Mat4 T = scaraFK(js);
    return {
        T.m[0][3],                             // x
        T.m[1][3],                             // y
        T.m[2][3],                             // z
        js.theta1 + js.theta2 + js.theta4      // phi
    };
}

// ─── Inverse Kinematics ───────────────────────────────────────────────

bool scaraIK(const OpState& op, JointState& js, bool elbowUp)
{
    // Shift target back by base offset
    float px = op.x - BASE_OFFSET;
    float py = op.y;

    float r2 = px*px + py*py;
    float r  = sqrtf(r2);

    // Reachability check
    if (r > (L1 + L2) || r < fabsf(L1 - L2)) {
        ESP_LOGE(KIN_TAG, "Target out of reach: r=%.2f max=%.2f", r, L1+L2);
        return false;
    }

    // J2 — law of cosines
    float cosT2 = (r2 - L1*L1 - L2*L2) / (2.0f * L1 * L2);
    // clamp to [-1, 1] to avoid NaN from floating point errors
    if (cosT2 >  1.0f) cosT2 =  1.0f;
    if (cosT2 < -1.0f) cosT2 = -1.0f;

    float sinT2  = elbowUp ? sqrtf(1.0f - cosT2*cosT2)
                           : -sqrtf(1.0f - cosT2*cosT2);
    js.theta2 = rad2deg(atan2f(sinT2, cosT2));

    // J1
    float k1  = L1 + L2 * cosT2;
    float k2  = L2 * sinT2;
    js.theta1 = rad2deg(atan2f(py, px) - atan2f(k2, k1));

    // Z is direct
    js.d3 = op.z;

    // J3 — maintain desired end-effector orientation
    js.theta4 = op.phi - js.theta1 - js.theta2;

    return true;
}

// ─── Motor conversion helpers ─────────────────────────────────────────

long angleToStepsJ1(float angleDeg)
{
    return (long)((angleDeg / 360.0f) * STEPS_PER_REV * MICROSTEP_J1 * GEAR_RATIO_J1);
}

long distToStepsZ(float mm)
{
    return (long)((mm / LEADSCREW_PITCH) * STEPS_PER_REV * MICROSTEP_Z);
}

long angleToCountsJ2(float angleDeg)
{
    return (long)((angleDeg / 360.0f) * ENC_CPR_J2 * GEAR_RATIO_J2);
}

long angleToCountsJ3(float angleDeg)
{
    return (long)((angleDeg / 360.0f) * ENC_CPR_J3 * GEAR_RATIO_J3);
}