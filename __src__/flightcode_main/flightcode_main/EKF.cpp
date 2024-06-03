#include "EKF.h"
#include <cmath>

using namespace Eigen;

EKF::EKF() {
    Vector4d quaternion(1, 0, 0, 0);
    Vector3d bias(0, 0, 0);

    xHat.resize(7);
    xHat << quaternion, bias;
    yHatBar.resize(3);
    yHatBar.setZero();
    p = MatrixXd::Identity(7, 7) * 0.01;
    Q = MatrixXd::Identity(7, 7) * 0.001;
    R = MatrixXd::Identity(6, 6) * 0.1;
    accelReference << 0, 0, -1;
    magReference << 0, -1, 0;
    mag_Ainv << 2.06423128e-03, -1.04778851e-04, -1.09416190e-06,
                -1.04778851e-04, 1.91693168e-03, 1.79409312e-05,
                -1.09416190e-06, 1.79409312e-05, 1.99819154e-03;
    mag_b << 80.51340236, 37.08931099, 105.6731885;
}

Vector4d EKF::normalizeQuat(Vector4d q) {
    return q / q.norm();
}

Vector3d EKF::getAccelVector(Vector3d a) {
    return a / a.norm();
}

Vector3d EKF::getMagVector(Vector3d m) {
    Vector3d magGaussRaw = mag_Ainv * (m - mag_b);
    Vector3d magGauss_N = getRotMat(xHat.head<4>()) * magGaussRaw;
    magGauss_N(2) = 0;
    return (getRotMat(xHat.head<4>()).transpose() * magGauss_N).normalized();
}

Matrix3d EKF::getRotMat(Vector4d q) {
    double c00 = q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3);
    double c01 = 2 * (q(1) * q(2) - q(0) * q(3));
    double c02 = 2 * (q(1) * q(3) + q(0) * q(2));
    double c10 = 2 * (q(1) * q(2) + q(0) * q(3));
    double c11 = q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3);
    double c12 = 2 * (q(2) * q(3) - q(0) * q(1));
    double c20 = 2 * (q(1) * q(3) - q(0) * q(2));
    double c21 = 2 * (q(2) * q(3) + q(0) * q(1));
    double c22 = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);

    Matrix3d rotMat;
    rotMat << c00, c01, c02,
              c10, c11, c12,
              c20, c21, c22;

    return rotMat;
}

MatrixXd EKF::getJacobianMatrix(Vector3d reference) {
    Vector4d qHatPrev = xHatPrev.head<4>();
    MatrixXd jacobianMatrix(3, 4);
    jacobianMatrix <<
        qHatPrev(0) * reference(0) + qHatPrev(3) * reference(1) - qHatPrev(2) * reference(2),
        qHatPrev(1) * reference(0) + qHatPrev(2) * reference(1) + qHatPrev(3) * reference(2),
        -qHatPrev(2) * reference(0) + qHatPrev(1) * reference(1) - qHatPrev(0) * reference(2),
        -qHatPrev(3) * reference(0) + qHatPrev(0) * reference(1) + qHatPrev(1) * reference(2),
        -qHatPrev(3) * reference(0) + qHatPrev(0) * reference(1) + qHatPrev(1) * reference(2),
        qHatPrev(2) * reference(0) - qHatPrev(1) * reference(1) + qHatPrev(0) * reference(2),
        qHatPrev(1) * reference(0) + qHatPrev(2) * reference(1) + qHatPrev(3) * reference(2),
        -qHatPrev(0) * reference(0) - qHatPrev(3) * reference(1) + qHatPrev(2) * reference(2),
        qHatPrev(2) * reference(0) - qHatPrev(1) * reference(1) + qHatPrev(0) * reference(2),
        qHatPrev(3) * reference(0) - qHatPrev(0) * reference(1) - qHatPrev(1) * reference(2),
        qHatPrev(0) * reference(0) + qHatPrev(3) * reference(1) - qHatPrev(2) * reference(2),
        qHatPrev(1) * reference(0) + qHatPrev(2) * reference(1) + qHatPrev(3) * reference(2);
    return 2 * jacobianMatrix;
}

VectorXd EKF::predictAccelMag() {
    MatrixXd hPrime_a = getJacobianMatrix(accelReference);
    Vector3d accelBar = getRotMat(xHatBar.head<4>()).transpose() * accelReference;
    MatrixXd hPrime_m = getJacobianMatrix(magReference);
    Vector3d magBar = getRotMat(xHatBar.head<4>()).transpose() * magReference;

    MatrixXd tmp1(3, 7);
    tmp1 << hPrime_a, MatrixXd::Zero(3, 3);
    MatrixXd tmp2(3, 7);
    tmp2 << hPrime_m, MatrixXd::Zero(3, 3);
    C.resize(6, 7);
    C << tmp1, tmp2;

    VectorXd result(6);
    result << accelBar, magBar;
    return result;
}

void EKF::predict(Vector3d w, double dt) {
    Vector4d q = xHat.head<4>();
    MatrixXd Sq(4, 3);
    Sq << -q(1), -q(2), -q(3),
           q(0), -q(3),  q(2),
           q(3),  q(0), -q(1),
          -q(2),  q(1),  q(0);
    MatrixXd tmp1(4, 7);
    tmp1 << MatrixXd::Identity(4, 4), -dt / 2 * Sq;
    MatrixXd tmp2(3, 7);
    tmp2 << MatrixXd::Zero(3, 4), MatrixXd::Identity(3, 3);
    A.resize(7, 7);
    A << tmp1, tmp2;
    B.resize(7, 3);
    B << dt / 2 * Sq, MatrixXd::Zero(3, 3);

    xHatBar = A * xHat + B * w;
    xHatBar.head<4>() = normalizeQuat(xHatBar.head<4>());
    xHatPrev = xHat;

    yHatBar = predictAccelMag();
    p = A * p * A.transpose() + Q;
}

void EKF::update(Vector3d a, Vector3d m) {
    MatrixXd tmp1 = (C * p * C.transpose() + R).inverse();
    K = p * C.transpose() * tmp1;

    Vector3d magGuass_B = getMagVector(m);
    Vector3d accel_B = getAccelVector(a);

    VectorXd measurement(6);
    measurement << accel_B, magGuass_B;
    xHat = xHatBar + K * (measurement - yHatBar);
    xHat.head<4>() = normalizeQuat(xHat.head<4>());
    p = (MatrixXd::Identity(7, 7) - K * C) * p;
}

Vector3d EKF::getEulerAngles(Vector4d q) {
    Matrix3d m = getRotMat(q);
    double test = -m(2, 0);
    // (Rest of your getEulerAngles code)
}

VectorXd EKF::getXHat() const {
    return xHat;
}