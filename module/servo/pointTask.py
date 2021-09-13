# _*_ coding:utf-8 _*_
# @time: 2021/3/24 下午8:39
# @author: 张新新
# @email: 1262981714@qq.com
import transforms3d as t3d
import random
import numpy as np


class Task():
    def __init__(self):
        self.gain = 0  # visual servoing gain
        # self.setInteractionMatrixType = 0
        self.dFeatures = None
        self.cFeatures = None
        self.eMc = None  # handeye parameter
        self.simu_eMc = None
        self.interactionMatrixType = 0
        self.Le = None
        self.error = None
        self.beta = 0

    def getError(self):
        if self.error is None and self.cFeatures is not None and self.dFeatures is not None:
            self.updateFeatureError()
        return self.error

    def getInteractionMatrix(self):
        return self.Le

    def setLambda(self, gain):
        '''
        设置伺服增益
        Args:
            gain:

        Returns:

        '''
        self.gain = gain

    def setBeta(self, beta):
        self.beta = beta

    def setInteractionMatrixType(self, interactionMatrixType):
        self.interactionMatrixType = interactionMatrixType

    def setDFeature(self, dFeatures):
        '''
        设置目标feature
        Args:
            dFeature: list<Module.feature>

        Returns:

        '''
        self.dFeatures = dFeatures

    def setCFeature(self, cFeatures):
        '''
        设置当前的feature
        Returns:

        '''
        self.cFeatures = cFeatures

    def setHandeye(self, eMc):
        '''
        设置手眼参数 eMc表示相机坐标系变换到机器臂末端的变换
        Args:
            eMc:

        Returns:

        '''
        self.eMc = eMc
        self.simu_eMc = eMc

    def addHandeyeNoise(self, euler_sigma, t_sigma):
        '''
        为手眼添加噪声
        Args:
            euler_sigma:
            t_sigma:

        Returns:

        '''
        if self.simu_eMc is None:
            print("handeye is None!")
            exit(0)
        euler = t3d.euler.mat2euler(self.simu_eMc)
        euler0 = euler[0] + random.gauss(0, euler_sigma)
        euler1 = euler[1] + random.gauss(0, euler_sigma)
        euler2 = euler[2] + random.gauss(0, euler_sigma)
        self.simu_eMc[:3, :3] = t3d.euler.euler2mat(euler0, euler1, euler2)
        self.simu_eMc[0, 3] = self.simu_eMc[0, 3] + random.gauss(0, t_sigma)
        self.simu_eMc[1, 3] = self.simu_eMc[1, 3] + random.gauss(0, t_sigma)
        self.simu_eMc[2, 3] = self.simu_eMc[2, 3] + random.gauss(0, t_sigma)

    def updateInteractionCurrent(self):
        '''
        Le
        Returns:

        '''
        n = len(self.cFeatures)
        self.Le = np.zeros([2 * n, 6])
        for i in range(n):
            x = self.cFeatures[i].x
            y = self.cFeatures[i].y
            Z = self.cFeatures[i].Z
            self.Le[2 * i, :] = np.array([-1 / Z, 0, x / Z, x * y, -(1 + x * x), y])
            self.Le[2 * i + 1, :] = np.array([0, -1 / Z, y / Z, 1 + y * y, -x * y, -x])

    def updateInteractionDestination(self):
        '''
        Le*
        Returns:

        '''
        n = len(self.dFeatures)
        self.Le = np.zeros([2 * n, 6])
        for i in range(n):
            x = self.dFeatures[i].x
            y = self.dFeatures[i].y
            Z = self.dFeatures[i].Z
            self.Le[2 * i, :] = np.array([-1 / Z, 0, x / Z, x * y, -(1 + x * x), y])
            self.Le[2 * i + 1, :] = np.array([0, -1 / Z, y / Z, 1 + y * y, -x * y, -x])

    def updateInteraction(self):
        if self.interactionMatrixType == 0:
            self.updateInteractionCurrent()
        else:
            self.updateInteractionDestination()

    def updateFeatureError(self):
        n = len(self.cFeatures)
        self.error = np.zeros([2 * n, 1])
        for i in range(n):
            self.error[2 * i, 0] = self.cFeatures[i].x - self.dFeatures[i].x
            self.error[2 * i + 1, 0] = self.cFeatures[i].y - self.dFeatures[i].y

    def getFeature(self):
        return self.error

    def getCameraVelocity(self):
        self.updateFeatureError()
        self.updateInteraction()
        return self.gain * np.dot(np.linalg.pinv(self.Le), self.error).flatten()

    def transformCameraToRobotVelocity(self, cameraVelocity):
        matrix = np.eye(4)
        matrix[:3, :3] = t3d.euler.euler2mat(cameraVelocity[3], cameraVelocity[4], cameraVelocity[5])
        matrix[:3, 3] = cameraVelocity[:3]
        matrix = np.dot(self.simu_eMc, np.dot(matrix, np.linalg.inv(self.simu_eMc)))
        euler = t3d.euler.mat2euler(matrix[:3, :3])
        return np.array([matrix[0, 3], matrix[1, 3], matrix[2, 3], euler[0], euler[1], euler[2]])

    def getCameraPoseFromRobot(self, bMe):
        return np.dot(bMe, self.eMc)





