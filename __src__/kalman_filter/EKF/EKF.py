# This code is gotten from: https://github.com/mbercas/attitude_bno055/blob/main/angle_measurement/Kalman_EKF.py

import numpy as np

""" Extended Kalman filter for attitude detection.

  - Original code from: https://thepoorengineer.com/wp-content/uploads/2018/09/PythonIMU_EKF.zip
  - Mostly unmodified
"""

def rad2deg(rad):
    return rad / np.pi * 180

def deg2rad(deg):
    return deg / 180 * np.pi

def getRotMat(q):
    c00 = q[0] ** 2 + q[1] ** 2 - q[2] ** 2 - q[3] ** 2
    c01 = 2 * (q[1] * q[2] - q[0] * q[3])
    c02 = 2 * (q[1] * q[3] + q[0] * q[2])
    c10 = 2 * (q[1] * q[2] + q[0] * q[3])
    c11 = q[0] ** 2 - q[1] ** 2 + q[2] ** 2 - q[3] ** 2
    c12 = 2 * (q[2] * q[3] - q[0] * q[1])
    c20 = 2 * (q[1] * q[3] - q[0] * q[2])
    c21 = 2 * (q[2] * q[3] + q[0] * q[1])
    c22 = q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2

    rotMat = np.array([[c00, c01, c02], [c10, c11, c12], [c20, c21, c22]])
    return rotMat

def getEulerAngles(q):
    m = getRotMat(q)
    test = -m[2, 0]
    if test > 0.99999:
        yaw = 0
        pitch = np.pi / 2
        roll = np.arctan2(m[0, 1], m[0, 2])
    elif test < -0.99999:
        yaw = 0
        pitch = -np.pi / 2
        roll = np.arctan2(-m[0, 1], -m[0, 2])
    else:
        yaw = np.arctan2(m[1, 0], m[0, 0])
        pitch = np.arcsin(-m[2, 0])
        roll = np.arctan2(m[2, 1], m[2, 2])

    yaw = rad2deg(yaw)
    pitch = rad2deg(pitch)
    roll = rad2deg(roll)

    return yaw, pitch, roll

class System:
    def __init__(self):
        quaternion = np.array([1, 0, 0, 0])     # Initial estimate of the quaternion
        bias = np.array([0, 0, 0])              # Initial estimate of the gyro bias

        self.xHat = np.concatenate((quaternion, bias)).transpose()
        self.yHatBar = np.zeros(3).transpose()
        self.p = np.identity(7) * 0.01
        self.Q = np.identity(7) * 0.001
        self.R = np.identity(6) * 0.1
        self.K = None
        self.A= None
        self.B = None
        self.C = None
        self.xHatBar = None
        self.xHatPrev = None
        self.pBar = None
        self.accelReference = np.array([0, 0, -1]).transpose()
        self.magReference = np.array([0, -1, 0]).transpose()
        self.mag_Ainv = np.array([[ 2.06423128e-03, -1.04778851e-04, -1.09416190e-06],
                                  [-1.04778851e-04,  1.91693168e-03,  1.79409312e-05],
                                  [-1.09416190e-06,  1.79409312e-05,  1.99819154e-03]])
        self.mag_b = np.array([80.51340236, 37.08931099, 105.6731885]).transpose()

    def normalizeQuat(self, q):
        mag = (q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)**0.5
        return q / mag

    def getAccelVector(self, a):
        accel = np.array(a).transpose()
        accelMag = (accel[0] ** 2 + accel[1] ** 2 + accel[2] ** 2) ** 0.5
        return accel / accelMag

    def getMagVector(self, m):
        magGaussRaw = np.matmul(self.mag_Ainv, np.array(m).transpose() - self.mag_b)
        magGauss_N = np.matmul(getRotMat(self.xHat), magGaussRaw)
        magGauss_N[2] = 0
        magGauss_N = magGauss_N / (magGauss_N[0] ** 2 + magGauss_N[1] ** 2) ** 0.5
        magGuass_B = np.matmul(getRotMat(self.xHat).transpose(), magGauss_N)
        return magGuass_B

    def getJacobianMatrix(self, reference):
        qHatPrev = self.xHatPrev[0:4]
        e00 = qHatPrev[0] * reference[0] + qHatPrev[3] * reference[1] - qHatPrev[2] * reference[2]
        e01 = qHatPrev[1] * reference[0] + qHatPrev[2] * reference[1] + qHatPrev[3] * reference[2]
        e02 = -qHatPrev[2] * reference[0] + qHatPrev[1] * reference[1] - qHatPrev[0] * reference[2]
        e03 = -qHatPrev[3] * reference[0] + qHatPrev[0] * reference[1] + qHatPrev[1] * reference[2]
        e10 = -qHatPrev[3] * reference[0] + qHatPrev[0] * reference[1] + qHatPrev[1] * reference[2]
        e11 = qHatPrev[2] * reference[0] - qHatPrev[1] * reference[1] + qHatPrev[0] * reference[2]
        e12 = qHatPrev[1] * reference[0] + qHatPrev[2] * reference[1] + qHatPrev[3] * reference[2]
        e13 = -qHatPrev[0] * reference[0] - qHatPrev[3] * reference[1] + qHatPrev[2] * reference[2]
        e20 = qHatPrev[2] * reference[0] - qHatPrev[1] * reference[1] + qHatPrev[0] * reference[2]
        e21 = qHatPrev[3] * reference[0] - qHatPrev[0] * reference[1] - qHatPrev[1] * reference[2]
        e22 = qHatPrev[0] * reference[0] + qHatPrev[3] * reference[1] - qHatPrev[2] * reference[2]
        e23 = qHatPrev[1] * reference[0] + qHatPrev[2] * reference[1] + qHatPrev[3] * reference[2]
        jacobianMatrix = 2 * np.array([[e00, e01, e02, e03],
                                       [e10, e11, e12, e13],
                                       [e20, e21, e22, e23]])
        return jacobianMatrix

    def predictAccelMag(self):
        # Accel
        hPrime_a = self.getJacobianMatrix(self.accelReference)
        accelBar = np.matmul(getRotMat(self.xHatBar).transpose(), self.accelReference)
        #print(accelBar)

        # Mag
        hPrime_m = self.getJacobianMatrix(self.magReference)
        magBar = np.matmul(getRotMat(self.xHatBar).transpose(), self.magReference)
        #print(magBar)

        tmp1 = np.concatenate((hPrime_a, np.zeros((3, 3))), axis=1)
        tmp2 = np.concatenate((hPrime_m, np.zeros((3, 3))), axis=1)
        self.C = np.concatenate((tmp1, tmp2), axis=0)

        return np.concatenate((accelBar, magBar), axis=0)

    def predict(self, w, dt):
        # xHat contains concatenates q & qB (quaternion of position & bias)
        # we are going to predict both...
        q = self.xHat[0:4]
        Sq = np.array([[-q[1], -q[2], -q[3]],
                       [ q[0], -q[3],  q[2]],
                       [ q[3],  q[0], -q[1]],
                       [-q[2],  q[1],  q[0]]])
        tmp1 = np.concatenate((np.identity(4), -dt / 2 * Sq), axis=1)
        tmp2 = np.concatenate((np.zeros((3, 4)), np.identity(3)), axis=1)
        self.A = np.concatenate((tmp1, tmp2), axis=0)
        self.B = np.concatenate((dt / 2 * Sq, np.zeros((3, 3))), axis=0)
        self.xHatBar = np.matmul(self.A, self.xHat) + np.matmul(self.B, np.array(w).transpose())
        self.xHatBar[0:4] = self.normalizeQuat(self.xHatBar[0:4])
        self.xHatPrev = self.xHat

        self.yHatBar = self.predictAccelMag()
        self.pBar = np.matmul(np.matmul(self.A, self.p), self.A.transpose()) + self.Q

    def update(self, a, m):
        tmp1 = np.linalg.inv(np.matmul(np.matmul(self.C, self.pBar), self.C.transpose()) + self.R)
        self.K = np.matmul(np.matmul(self.pBar, self.C.transpose()), tmp1)

        magGuass_B = self.getMagVector(m)
        accel_B = self.getAccelVector(a)

        measurement = np.concatenate((accel_B, magGuass_B), axis=0)
        self.xHat = self.xHatBar + np.matmul(self.K, measurement - self.yHatBar)
        self.xHat[0:4] = self.normalizeQuat(self.xHat[0:4])
        self.p = np.matmul(np.identity(7) - np.matmul(self.K, self.C), self.pBar)