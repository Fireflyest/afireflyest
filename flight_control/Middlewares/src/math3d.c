#include "math3d.h"
#include "gfx.h" // For screen center
#include "math.h"
#include "arm_math.h"


static inline int round_to_int(float v) {
    return (int)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

void Math3D_Project(Point3D* p, float focalLength, float cameraZ) {
    float scale = focalLength / (p->z + cameraZ);
    p->x = round_to_int(p->x * scale) + OLED_WIDTH / 2;
    p->y = round_to_int(p->y * scale) + OLED_HEIGHT / 2;
}

Quaternion Math3D_QuatFromEuler(float yaw, float pitch, float roll) {
    float cy = arm_cos_f32(yaw * 0.5f), sy = arm_sin_f32(yaw * 0.5f);
    float cp = arm_cos_f32(pitch * 0.5f), sp = arm_sin_f32(pitch * 0.5f);
    float cr = arm_cos_f32(roll * 0.5f), sr = arm_sin_f32(roll * 0.5f);
    Quaternion q;
    q.w = cr*cp*cy + sr*sp*sy;
    q.x = sr*cp*cy - cr*sp*sy;
    q.y = cr*sp*cy + sr*cp*sy;
    q.z = cr*cp*sy - sr*sp*cy;
    return q;
}

Quaternion Math3D_QuatFromAxisAngle(Vector3D axis, float angle) {
    float s = arm_sin_f32(angle * 0.5f);
    Quaternion q;
    q.w = arm_cos_f32(angle * 0.5f);
    q.x = axis.x * s;
    q.y = axis.y * s;
    q.z = axis.z * s;
    return q;
}

// Fast rotate vector p by quaternion q (uses optimized cross-product form)
Point3D Math3D_RotateByQuat(Point3D v, Quaternion q) {
    // t = 2 * cross(q_vec, v)
    float tx = 2.0f * (q.y * v.z - q.z * v.y);
    float ty = 2.0f * (q.z * v.x - q.x * v.z);
    float tz = 2.0f * (q.x * v.y - q.y * v.x);
    // v' = v + q.w * t + cross(q_vec, t)
    float cx = q.y * tz - q.z * ty;
    float cy = q.z * tx - q.x * tz;
    float cz = q.x * ty - q.y * tx;
    Point3D out;
    out.x = v.x + q.w * tx + cx;
    out.y = v.y + q.w * ty + cy;
    out.z = v.z + q.w * tz + cz;
    return out;
}

void Math3D_QuatRotateVector(Vector3D* v, Quaternion* q) {
    // t = 2 * cross(q_vec, v)
    float tx = 2.0f * (q->y * v->z - q->z * v->y);
    float ty = 2.0f * (q->z * v->x - q->x * v->z);
    float tz = 2.0f * (q->x * v->y - q->y * v->x);
    // v' = v + q.w * t + cross(q_vec, t)
    float cx = q->y * tz - q->z * ty;
    float cy = q->z * tx - q->x * tz;
    float cz = q->x * ty - q->y * tx;
    v->x = v->x + q->w * tx + cx;
    v->y = v->y + q->w * ty + cy;
    v->z = v->z + q->w * tz + cz;
}

void Math3D_VectorNormalize(Vector3D* v) {
    float n = 0;
    arm_sqrt_f32(v->x*v->x + v->y*v->y + v->z*v->z, &n);
    if (n <= 0.0f) return;
    float inv = 1.0f / n;
    v->x *= inv; v->y *= inv; v->z *= inv;
    return;
}

void Math3D_VectorMultiplyScalar(Vector3D* v, float s) {
    v->x *= s;
    v->y *= s;
    v->z *= s;
}


void Math3D_QuatNormalize(Quaternion* q) {
    float n = 0;
    arm_sqrt_f32(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z, &n);
    if (n <= 0.0f) return;
    float inv = 1.0f / n;
    q->w *= inv; q->x *= inv; q->y *= inv; q->z *= inv;
    return;
}

void Math3D_QuatConjugate(Quaternion* q) {
    q->x = -q->x;
    q->y = -q->y;
    q->z = -q->z;
}

void Math3D_QuatMultiply(Quaternion* q1, Quaternion* q2) {
    Quaternion result;
    result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
    *q1 = result;
}

void Math3D_QuatMultiply_f32(float q1[4], const float q2[4]) {
    float w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    float x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    float y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    float z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
    q1[0] = w;
    q1[1] = x;
    q1[2] = y;
    q1[3] = z;
}

Point3D Math3D_RotateX(Point3D p, float angle) {
    Point3D newP;
    float c = arm_cos_f32(angle);
    float s = arm_sin_f32(angle);
    newP.x = p.x;
    newP.y = p.y * c - p.z * s;
    newP.z = p.y * s + p.z * c;
    return newP;
}

Point3D Math3D_RotateY(Point3D p, float angle) {
    Point3D newP;
    float c = arm_cos_f32(angle);
    float s = arm_sin_f32(angle);
    newP.x = p.x * c + p.z * s;
    newP.y = p.y;
    newP.z = -p.x * s + p.z * c;
    return newP;
}

Point3D Math3D_RotateZ(Point3D p, float angle) {
    Point3D newP;
    float c = arm_cos_f32(angle);
    float s = arm_sin_f32(angle);
    newP.x = p.x * c - p.y * s;
    newP.y = p.x * s + p.y * c;
    newP.z = p.z;
    return newP;
}

