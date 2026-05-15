#include "spatial_math.h"
#include "math.h"
#include "arm_math.h"

float Spatial_InvSqrt(float x) {
    union {
        float f;
        int i;
    } conv;
    float xhalf = 0.5f * x;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    x = conv.f;
    x = x * (1.5f - xhalf * x * x); // 一次牛顿迭代
    return x;
}


void Spatial_ProjectTo2D(sm_vec2_t out, const sm_vec3_t in, float focalLength, float cameraZ, float screenWidth, float screenHeight) {
    float z_eff = in[2] + cameraZ;
    z_eff = fmaxf(z_eff, 0.000001f);    
    float scale = focalLength / z_eff;
    out[0] = (in[0] * scale) + (screenWidth * 0.5f);
    out[1] = (in[1] * scale) + (screenHeight * 0.5f);
}

void Spatial_RotatePointByQuat(sm_vec3_t out, const sm_vec3_t point, const sm_quat_t q) {
    // q = [w, x, y, z] -> [q0, q1, q2, q3]
    // t = 2 * cross(q_vec, point)
    float tx = 2.0f * (q[2] * point[2] - q[3] * point[1]);
    float ty = 2.0f * (q[3] * point[0] - q[1] * point[2]);
    float tz = 2.0f * (q[1] * point[1] - q[2] * point[0]);

    // out = point + q.w * t + cross(q_vec, t)
    out[0] = point[0] + q[0] * tx + (q[2] * tz - q[3] * ty);
    out[1] = point[1] + q[0] * ty + (q[3] * tx - q[1] * tz);
    out[2] = point[2] + q[0] * tz + (q[1] * ty - q[2] * tx);
}

void Spatial_Vec3Normalize(sm_vec3_t v) {
    float magSq = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
    if (magSq < 0.0000001f) return;
    
    float inv = Spatial_InvSqrt(magSq);
    v[0] *= inv; v[1] *= inv; v[2] *= inv;
}

void Spatial_Vec3MultiplyScalar(sm_vec3_t v, float s) {
    v[0] *= s; v[1] *= s; v[2] *= s;
}

void Spatial_Vec3Cross(sm_vec3_t out, const sm_vec3_t a, const sm_vec3_t b) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

float Spatial_Vec3Dot(const sm_vec3_t a, const sm_vec3_t b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void Spatial_Vec3Add(sm_vec3_t out, const sm_vec3_t a, const sm_vec3_t b) {
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
}

void Spatial_Vec3Sub(sm_vec3_t out, const sm_vec3_t a, const sm_vec3_t b) {
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

float Spatial_Vec3IsZero(const sm_vec3_t v) {
    return (v[0]*v[0] + v[1]*v[1] + v[2]*v[2] < 1e-12f);
}


void Spatial_QuatFromEuler(sm_quat_t out, float yaw, float pitch, float roll) {
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    out[0] = cr * cp * cy + sr * sp * sy; // w
    out[1] = sr * cp * cy - cr * sp * sy; // x
    out[2] = cr * sp * cy + sr * cp * sy; // y
    out[3] = cr * cp * sy - sr * sp * cy; // z
}

void Spatial_QuatNormalize(sm_quat_t q) {
    float magSq = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (magSq < 0.0000001f) return;
    
    float inv = Spatial_InvSqrt(magSq);
    q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
}

void Spatial_Vec3RotateByQuat(sm_vec3_t v, const sm_quat_t q) {
    sm_vec3_t temp;
    Spatial_RotatePointByQuat(temp, v, q);
    v[0] = temp[0]; v[1] = temp[1]; v[2] = temp[2];
}

void Spatial_QuatConjugate(sm_quat_t q) {
    q[1] = -q[1];
    q[2] = -q[2];
    q[3] = -q[3];
}

void Spatial_QuatMultiply(sm_quat_t out, const sm_quat_t q1, const sm_quat_t q2) {
    out[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    out[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    out[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    out[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

void Spatial_QuatGetEuler(float *yaw, float *pitch, float *roll, const sm_quat_t q) {
    float q00 = q[0]*q[0], q11 = q[1]*q[1], q22 = q[2]*q[2], q33 = q[3]*q[3];
    
    // yaw (Z-axis rotation)
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), q00 + q11 - q22 - q33);
    
    // pitch (Y-axis rotation)
    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(SM_PI * 0.5f, sinp);
    else
        *pitch = asinf(sinp);
        
    // roll (X-axis rotation)
    *roll = atan2f(2.0f * (q[1] * q[0] + q[2] * q[3]), q00 - q11 - q22 + q33);
}

void Spatial_QuatRotate(sm_quat_t out, float yaw, float pitch, float roll) {
    sm_quat_t rot;
    Spatial_QuatFromEuler(rot, yaw, pitch, roll);
    Spatial_QuatMultiply(out, rot, out);
    Spatial_QuatNormalize(out);
}

float Spatial_QuatAngleBetween(const sm_quat_t q1, const sm_quat_t q2) {
    float dot = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3];
    return acosf(fminf(fabsf(dot), 1.0f)) * 2.0f;
}
