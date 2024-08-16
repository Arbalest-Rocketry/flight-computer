#include "quaternion.h"
#include <math.h>

void eulerToQuaternion(float yaw, float pitch, float roll, Quaternion* q) {
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);

    q->w = cy * cp * cr + sy * sp * sr;
    q->x = cy * cp * sr - sy * sp * cr;
    q->y = sy * cp * sr + cy * sp * cr;
    q->z = sy * cp * cr - cy * sp * sr;

    normalizeQuaternion(q);
}

void normalizeQuaternion(Quaternion* q) {
    float norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    q->w /= norm;q->x /= norm;q->y /= norm;q->z /= norm;
}

void invertQuaternion(const Quaternion* q, Quaternion* result) {
    result->w = q->w; result->x = -q->x;result->y = -q->y;result->z = -q->z;
}

void multiplyQuaternions(const Quaternion* q1, const Quaternion* q2, Quaternion* result) {
    result->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

void quaternionToMatrix(const Quaternion* q, float matrix[3][3]) {
    float ww = q->w * q->w;
    float xx = q->x * q->x;
    float yy = q->y * q->y;
    float zz = q->z * q->z;

    matrix[0][0] = ww + xx - yy - zz;
    matrix[0][1] = 2.0f * (q->x * q->y - q->w * q->z);
    matrix[0][2] = 2.0f * (q->x * q->z + q->w * q->y);
    matrix[1][0] = 2.0f * (q->x * q->y + q->w * q->z);
    matrix[1][1] = ww - xx + yy - zz;
    matrix[1][2] = 2.0f * (q->y * q->z - q->w * q->x);
    matrix[2][0] = 2.0f * (q->x * q->z - q->w * q->y);
    matrix[2][1] = 2.0f * (q->y * q->z + q->w * q->x);
    matrix[2][2] = ww - xx - yy + zz;
}