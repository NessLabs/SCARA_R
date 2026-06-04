#include "RobotState.h"

// ─── Global instance ──────────────────────────────────────────────────
RobotState robotState = {
    .joints     = { 0.0f, 0.0f, 0.0f, 0.0f },
    .velocities = { 0.0f, 0.0f, 0.0f, 0.0f },
    .tcp        = { 0.0f, 0.0f, 0.0f, 0.0f },
    .estop      = false
};

// ─── Update from sensor readings ─────────────────────────────────────
void updateRobotState(float theta1, float theta2, float theta3, float d4,
                      float vel1,   float vel2,   float vel3,   float vel4)
{
    robotState.joints.theta1 = theta1;
    robotState.joints.theta2 = theta2;
    robotState.joints.theta3 = theta3;
    robotState.joints.d4     = d4;

    robotState.velocities.theta1 = vel1;
    robotState.velocities.theta2 = vel2;
    robotState.velocities.theta3 = vel3;
    robotState.velocities.d4     = vel4;

    updateFK();
}

// ─── Recompute TCP from joints ────────────────────────────────────────
void updateFK()
{
    robotState.tcp = getOpState(robotState.joints);
}