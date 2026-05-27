#include "Kinematics.h"
#include "esp_log.h"
#include <math.h>

static const char* KIN_TAG = "Kinematics";

// ─── Forward Kinematics ───────────────────────────────────────────────
// J1 and J2 move the arm in XY (fixed height)
// J3 rotates the gripper
// d4 moves only the gripper in Z

Mat4 scaraFK(const JointState& js)
{
    // DH table: {theta, d, r, alpha}
    DHParams dh1 = { js.theta1, 0,      L1, 0 };  // J1 rotation
    DHParams dh2 = { js.theta2, 0,      L2, 0 };  // J2 rotation
    DHParams dh3 = { js.theta3, 0,      0,  0 };  // gripper rotation
    DHParams dh4 = { 0,         js.d4,  0,  0 };  // gripper Z (prismatic)

    Mat4 T1 = getHM(dh1);
    Mat4 T2 = getHM(dh2);
    Mat4 T3 = getHM(dh3);
    Mat4 T4 = getHM(dh4);

    return mat4Mul(mat4Mul(mat4Mul(T1, T2), T3), T4);
}

OpState getOpState(const JointState& js)
{
    Mat4 T = scaraFK(js);
    return {
        T.m[0][3],                              // x
        T.m[1][3],                              // y
        T.m[2][3],                              // z
        js.theta1 + js.theta2 + js.theta3       // phi
    };
}

// ─── Inverse Kinematics ───────────────────────────────────────────────

bool scaraIK(const OpState& op, JointState& js, bool elbowUp)
{
    float px = op.x;
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
    if (cosT2 >  1.0f) cosT2 =  1.0f;
    if (cosT2 < -1.0f) cosT2 = -1.0f;

    float sinT2  = elbowUp ? sqrtf(1.0f - cosT2*cosT2)
                           : -sqrtf(1.0f - cosT2*cosT2);
    js.theta2 = rad2deg(atan2f(sinT2, cosT2));

    // J1
    float k1  = L1 + L2 * cosT2;
    float k2  = L2 * sinT2;
    js.theta1 = rad2deg(atan2f(py, px) - atan2f(k2, k1));

    // Z is direct — only gripper moves
    js.d4 = op.z;

    // Gripper rotation — maintain desired end-effector orientation
    js.theta3 = op.phi - js.theta1 - js.theta2;

    return true;
}

// ─── Motor conversion helpers ─────────────────────────────────────────

long angleToStepsJ1(float angleDeg)
{
    return (long)((angleDeg / 360.0f) * STEPS_PER_REV * MICROSTEP_J1 * GEAR_RATIO_J1);
}

long angleToStepsJ2(float angleDeg)
{
    return (long)((angleDeg / 360.0f) * STEPS_PER_REV * MICROSTEP_J2 * GEAR_RATIO_J2);
}

long distToCountsZ(float mm)
{
    return (long)((mm / MM_PER_REV_Z) * ENC_CPR_Z * GEAR_RATIO_Z);
}

long angleToCountsJ3(float angleDeg)
{
    return (long)((angleDeg / 360.0f) * ENC_CPR_J3 * GEAR_RATIO_J3);
}

// ─── inverseKinematics ────────────────────────────────────────────────
// Direct translation of MATLAB InverseKinematics()
// Returns up to 2 solutions, count = 0 if out of reach

IKSolutions inverseKinematics(const OpState& op)
{
    IKSolutions result;
    result.count = 0;

    float x = op.x;
    float y = op.y;
    float z = op.z;
    float toolAngle = op.phi;

    float P = sqrtf(x*x + y*y);

    // reachability check
    if (P > L1 + L2 || P < fabsf(L1 - L2))
        return result;

    float gamma = atan2f(y, x);
    float alpha = acosf((L1*L1 + P*P - L2*L2) / (2.0f * L1 * P));
    float beta  = acosf((L1*L1 + L2*L2 - P*P) / (2.0f * L1 * L2));

    // solution 1 — elbow up
    result.solutions[0].theta1 = rad2deg(gamma - alpha);
    result.solutions[0].theta2 = rad2deg((float)M_PI - beta);
    result.solutions[0].d4     = z;
    result.solutions[0].theta3 = toolAngle
                                 - result.solutions[0].theta1
                                 - result.solutions[0].theta2;
    result.count = 1;

    // singular case — only one solution
    if (P == L1 + L2 || P == fabsf(L1 - L2))
        return result;

    // solution 2 — elbow down
    result.solutions[1].theta1 = rad2deg(gamma + alpha);
    result.solutions[1].theta2 = rad2deg(beta - (float)M_PI);
    result.solutions[1].d4     = z;
    result.solutions[1].theta3 = toolAngle
                                 - result.solutions[1].theta1
                                 - result.solutions[1].theta2;
    result.count = 2;

    return result;
}

// ─── findBestSolution ─────────────────────────────────────────────────
// Direct translation of MATLAB findBestSolution()
// weights: importance of each joint {w_theta2, w_theta3, w_d4}
// returns index of best solution (0 or 1)

int findBestSolution(const IKSolutions& ik, const JointState& current,
                     float weights[3])
{
    float bestError = 1e12f;
    int   bestIndex = 0;

    for (int i = 0; i < ik.count; i++)
    {
        float err = 0.0f;
        err += weights[0] * fabsf(ik.solutions[i].theta2 - current.theta2);
        err += weights[1] * fabsf(ik.solutions[i].theta3 - current.theta3);
        err += weights[2] * fabsf(ik.solutions[i].d4     - current.d4);

        if (err < bestError)
        {
            bestError = err;
            bestIndex = i;
        }
    }

    return bestIndex;
}