#include "KinematicsMath.h"

// ─── Unit conversion ─────────────────────────────────────────────────

float deg2rad(float deg) { return deg * M_PI / 180.0f; }
float rad2deg(float rad) { return rad * 180.0f / M_PI; }

// ─── Rotation matrices ───────────────────────────────────────────────

Mat3 rotateAxis(char axis, float angleDeg)
{
    Mat3 R;
    float c = cosf(deg2rad(angleDeg));
    float s = sinf(deg2rad(angleDeg));

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R.m[i][j] = 0.0f;

    switch (axis) {
        case 'x':
            R.m[0][0]=1; R.m[1][1]=c; R.m[1][2]=-s; R.m[2][1]=s; R.m[2][2]=c;
            break;
        case 'y':
            R.m[0][0]=c; R.m[0][2]=s; R.m[1][1]=1; R.m[2][0]=-s; R.m[2][2]=c;
            break;
        case 'z':
            R.m[0][0]=c; R.m[0][1]=-s; R.m[1][0]=s; R.m[1][1]=c; R.m[2][2]=1;
            break;
    }
    return R;
}

// ─── Quaternion math ─────────────────────────────────────────────────

Vec4 composeQuat(Vec4 p, Vec4 q)
{
    return {
        p.w*q.w - p.x*q.x - p.y*q.y - p.z*q.z,
        p.w*q.x + p.x*q.w + p.y*q.z - p.z*q.y,
        p.w*q.y - p.x*q.z + p.y*q.w + p.z*q.x,
        p.w*q.z + p.x*q.y - p.y*q.x + p.z*q.w
    };
}

Vec3 rotateVectQuat(Vec4 q, Vec3 v)
{
    Vec4 qConj = { q.w, -q.x, -q.y, -q.z };
    Vec4 vQuat = { 0.0f, v.x, v.y, v.z };
    Vec4 q1    = composeQuat(q, vQuat);
    Vec4 vRot  = composeQuat(q1, qConj);
    return { vRot.x, vRot.y, vRot.z };
}

// ─── Homogeneous transforms ──────────────────────────────────────────

Mat4 getHM(DHParams dh)
{
    float ct = cosf(deg2rad(dh.theta));
    float st = sinf(deg2rad(dh.theta));
    float ca = cosf(deg2rad(dh.alpha));
    float sa = sinf(deg2rad(dh.alpha));

    Mat4 H;
    H.m[0][0]=ct;  H.m[0][1]=-ca*st;  H.m[0][2]=sa*st;  H.m[0][3]=dh.r*ct;
    H.m[1][0]=st;  H.m[1][1]= ca*ct;  H.m[1][2]=-sa*ct; H.m[1][3]=dh.r*st;
    H.m[2][0]=0;   H.m[2][1]= sa;     H.m[2][2]=ca;      H.m[2][3]=dh.d;
    H.m[3][0]=0;   H.m[3][1]= 0;      H.m[3][2]=0;       H.m[3][3]=1;
    return H;
}

Mat4 mat4Mul(const Mat4& A, const Mat4& B)
{
    Mat4 C;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            C.m[i][j] = 0.0f;
            for (int k = 0; k < 4; k++)
                C.m[i][j] += A.m[i][k] * B.m[k][j];
        }
    return C;
}

Mat3 mat3Mul(const Mat3& A, const Mat3& B)
{
    Mat3 C;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            C.m[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
                C.m[i][j] += A.m[i][k] * B.m[k][j];
        }
    return C;
}

Mat4 mat4Identity()
{
    Mat4 I;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            I.m[i][j] = (i == j) ? 1.0f : 0.0f;
    return I;
}