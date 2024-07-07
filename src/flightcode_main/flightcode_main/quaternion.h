#ifndef QUATERNION_H
#define QUATERNION_H

struct Quaternion {
    float w, x, y, z;
};

void eulerToQuaternion(float yaw, float pitch, float roll, Quaternion* q);
void normalizeQuaternion(Quaternion* q);
void invertQuaternion(const Quaternion* q, Quaternion* result);
void multiplyQuaternions(const Quaternion* q1, const Quaternion* q2, Quaternion* result);
void quaternionToMatrix(const Quaternion* q, float matrix[3][3]);

#endif /* QUATERNION_H */