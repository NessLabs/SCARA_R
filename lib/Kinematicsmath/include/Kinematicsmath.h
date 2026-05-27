#ifndef _KINEMATICSMATH_H
#define _KINEMATICSMATH_H

#include <math.h>

// ─── Types ───────────────────────────────────────────────────────────

struct Vec3 {
    float x, y, z;
};

struct Vec4 {   // quaternion [w, x, y, z]
    float w, x, y, z;
};

struct Mat3 {
    float m[3][3];
};

struct Mat4 {
    float m[4][4];
};

struct DHParams {
    float theta;  // joint angle (deg)
    float d;      // joint offset along Z (mm)
    float r;      // link length along X (mm)
    float alpha;  // link twist (deg)
};

// ─── Unit conversion ─────────────────────────────────────────────────

float deg2rad(float deg);
float rad2deg(float rad);

// ─── Rotation matrices ───────────────────────────────────────────────

Mat3 rotateAxis(char axis, float angleDeg);

// ─── Quaternion math ─────────────────────────────────────────────────

Vec4 composeQuat(Vec4 p, Vec4 q);
Vec3 rotateVectQuat(Vec4 q, Vec3 v);

// ─── Homogeneous transforms ──────────────────────────────────────────

Mat4 getHM(DHParams dh);
Mat4 mat4Mul(const Mat4& A, const Mat4& B);
Mat4 mat4Identity();
Mat3 mat3Mul(const Mat3& A, const Mat3& B);

#endif // _KINEMATICSMATH_H