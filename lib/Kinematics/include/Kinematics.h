#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include "KinematicsMath.h"

// ─── Robot physical constants ─────────────────────────────────────────

constexpr float L1          = 130.0f;  // link 1 length (mm)
constexpr float L2          = 130.0f;  // link 2 length (mm)

// ─── Types ────────────────────────────────────────────────────────────

struct JointState {
    float theta1;  // J1 angle (deg) — stepper, homed via limit switch
    float theta2;  // J2 angle (deg) — stepper, absolute encoder AS5600
    float theta3;  // gripper rotation (deg) — BDC + quadrature
    float d4;      // gripper Z position (mm) — BDC + quadrature
};

struct OpState {
    float x, y, z;  // end-effector position (mm)
    float phi;       // end-effector orientation (deg) = theta1 + theta2 + theta3
};

// Holds up to 2 IK solutions
struct IKSolutions {
    JointState solutions[2];
    int        count;       // 0 = out of reach, 1 = singular, 2 = normal
};

// ─── Functions ────────────────────────────────────────────────────────

Mat4    scaraFK(const JointState& js);
OpState getOpState(const JointState& js);

// Original single-solution IK (elbowUp flag)
bool scaraIK(const OpState& op, JointState& js, bool elbowUp = true);

// New: returns both IK solutions (matches MATLAB InverseKinematics)
IKSolutions inverseKinematics(const OpState& op);

// New: picks best solution based on weighted distance from current state
// weights: {w_theta2, w_theta3, w_d4} — importance of each joint
int findBestSolution(const IKSolutions& ik, const JointState& current,
                     float weights[3]);

// ─── Motor conversion helpers ─────────────────────────────────────────

constexpr int   STEPS_PER_REV   = 200;
constexpr int   MICROSTEP_J1    = 16;
constexpr int   MICROSTEP_J2    = 16;
constexpr float GEAR_RATIO_J1   = 1.0f;
constexpr float GEAR_RATIO_J2   = 1.0f;
constexpr float GEAR_RATIO_J3   = 1.0f;
constexpr float GEAR_RATIO_Z    = 1.0f;
constexpr int   ENC_CPR_J3      = 1000;
constexpr int   ENC_CPR_Z       = 1000;
constexpr float MM_PER_REV_Z    = 10.0f;

long angleToStepsJ1(float angleDeg);
long angleToStepsJ2(float angleDeg);
long distToCountsZ(float mm);
long angleToCountsJ3(float angleDeg);

#endif // _KINEMATICS_H