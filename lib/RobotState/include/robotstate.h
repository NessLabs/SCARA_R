#pragma once

#include "Kinematics.h"

// ─── Joint velocities ─────────────────────────────────────────────────
struct JointVelocity {
    float theta1;  // deg/s — from AS5600 delta
    float theta2;  // deg/s — from step rate
    float theta3;  // deg/s — from quadrature encoder
    float d4;      // mm/s  — from quadrature encoder
};

// ─── Full robot state ─────────────────────────────────────────────────
struct RobotState {
    JointState    joints;      // current joint positions
    JointVelocity velocities;  // current joint velocities
    OpState       tcp;         // end-effector position (computed via FK)
    bool          estop;       // emergency stop active
};

// ─── Global state instance ────────────────────────────────────────────
// Include this header in any file that needs to read/write robot state
extern RobotState robotState;

// ─── Update functions ─────────────────────────────────────────────────

// Call this every control cycle with fresh sensor readings
// Automatically recomputes FK and updates tcp
void updateRobotState(float theta1, float theta2, float theta3, float d4,
                      float vel1,   float vel2,   float vel3,   float vel4);

// Recompute TCP from current joint state
void updateFK();