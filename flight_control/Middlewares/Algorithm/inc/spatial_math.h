#ifndef __SPATIAL_MATH_H
#define __SPATIAL_MATH_H

#define SM_PI 3.14159265358979323846f
#define SM_HALF_PI 1.57079632679489661923f
#define SM_DEG2RAD (SM_PI / 180.0f)
#define SM_RAD2DEG (180.0f / SM_PI)
#define SM_G 9.81f

#define FOCAL_LENGTH 64.0f
#define CAMERA_Z    100.0f

typedef float sm_vec2_t[2];
typedef float sm_vec3_t[3];
typedef float sm_quat_t[4];
typedef float sm_mat4_t[16];

float Spatial_InvSqrt(float x);

void Spatial_ProjectTo2D(sm_vec2_t out, const sm_vec3_t in, float focalLength, float cameraZ, float screenWidth, float screenHeight);
void Spatial_RotatePointByQuat(sm_vec3_t out, const sm_vec3_t point, const sm_quat_t q);

void Spatial_Vec3Normalize(sm_vec3_t v);
void Spatial_Vec3MultiplyScalar(sm_vec3_t v, float s);
void Spatial_Vec3Cross(sm_vec3_t out, const sm_vec3_t a, const sm_vec3_t b);
float Spatial_Vec3Dot(const sm_vec3_t a, const sm_vec3_t b);
void Spatial_Vec3Add(sm_vec3_t out, const sm_vec3_t a, const sm_vec3_t b);
void Spatial_Vec3Sub(sm_vec3_t out, const sm_vec3_t a, const sm_vec3_t b);
float Spatial_Vec3IsZero(const sm_vec3_t v);
void Spatial_Vec3RotateByQuat(sm_vec3_t v, const sm_quat_t q);


void Spatial_QuatFromEuler(sm_quat_t out, float yaw, float pitch, float roll);
void Spatial_QuatNormalize(sm_quat_t q);
void Spatial_QuatConjugate(sm_quat_t q);
void Spatial_QuatMultiply(sm_quat_t out, const sm_quat_t q1, const sm_quat_t q2);
void Spatial_QuatGetEuler(float *yaw, float *pitch, float *roll, const sm_quat_t q);
void Spatial_QuatRotate(sm_quat_t out, float yaw, float pitch, float roll);
float Spatial_QuatAngleBetween(const sm_quat_t q1, const sm_quat_t q2);


#endif /* __SPATIAL_MATH_H */