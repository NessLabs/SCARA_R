#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include "KinematicsMath.h"

// ─── Robot physical constants ─────────────────────────────────────────
// Fill in TBD values once known

constexpr float BASE_OFFSET = 75.0f;   // distance from origin to J1 pivot (mm)
constexpr float L1          = 112.5f;  // link 1 length (mm)
constexpr float L2          = 112.5f;  // link 2 length (mm)

// ─── Types ────────────────────────────────────────────────────────────

struct JointState {
    float theta1;  // J1 angle (deg)
    float theta2;  // J2 angle (deg)
    float d3;      // Z position (mm)
    float theta4;  // J3 rotational angle (deg)
};

struct OpState {
    float x, y, z;  // end-effector position (mm)
    float phi;       // end-effector orientation (deg) = theta1+theta2+theta4
};

// ─── Functions ────────────────────────────────────────────────────────

// Forward kinematics: joint space → operational space
Mat4   scaraFK(const JointState& js);
OpState getOpState(const JointState& js);

// Inverse kinematics: operational space → joint space
// elbowUp: true = elbow up solution, false = elbow down
// returns false if target is out of reach
bool scaraIK(const OpState& op, JointState& js, bool elbowUp = true);

// ─── Motor conversion helpers ─────────────────────────────────────────
// Fill in TBD values once known

constexpr int   STEPS_PER_REV    = 200;   // NEMA17 full steps/rev
constexpr int   MICROSTEP_J1     = 16;    // TBD
constexpr int   MICROSTEP_Z      = 16;    // TBD
constexpr float GEAR_RATIO_J1    = 1.0f;  // TBD
constexpr float GEAR_RATIO_J2    = 1.0f;  // TBD
constexpr float GEAR_RATIO_J3    = 1.0f;  // TBD
constexpr float LEADSCREW_PITCH  = 8.0f;  // mm per motor revolution (TBD)
constexpr int   ENC_CPR_J2       = 1000;  // TBD
constexpr int   ENC_CPR_J3       = 1000;  // TBD

long  angleToStepsJ1(float angleDeg);
long  distToStepsZ(float mm);
long  angleToCountsJ2(float angleDeg);
long  angleToCountsJ3(float angleDeg);

#endif // _KINEMATICS_H
