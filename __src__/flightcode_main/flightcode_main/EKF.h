#ifndef EKF_H
#define EKF_H

#include <ArduinoEigenDense.h>
#include <cmath>

using namespace Eigen;

class EKF {
public:
    EKF();
    VectorXd getXHat() const;  // Add this line to declare the getter method.
    void predict(Vector3d w, double dt);
    void update(Vector3d a, Vector3d m);
    Vector3d getEulerAngles(Vector4d q);

private:
    Matrix3d getRotMat(Vector4d q);
    Vector4d normalizeQuat(Vector4d q);
    Vector3d getAccelVector(Vector3d a);
    Vector3d getMagVector(Vector3d m);
    MatrixXd getJacobianMatrix(Vector3d reference);
    VectorXd predictAccelMag();

    VectorXd xHat;
    VectorXd xHatBar;
    VectorXd xHatPrev;
    Vector3d yHatBar;
    MatrixXd p;
    MatrixXd Q;
    MatrixXd R;
    MatrixXd K;
    MatrixXd A;
    MatrixXd B;
    MatrixXd C;
    Vector3d accelReference;
    Vector3d magReference;
    Matrix3d mag_Ainv;
    Vector3d mag_b;
};

#endif /* EKF_H */