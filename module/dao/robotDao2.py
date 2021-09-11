# _*_ coding:utf-8 _*_
# @time: 2021/8/24 下午2:59
# @author: 张新新
# @email: 1262981714@qq.com

from abc import ABC,abstractmethod
import numpy as np
import time
import transforms3d as t3d
import threading

class robotDao(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.samplingTime = 0
        self.position = None
        self.velocity = np.array([0,0,0,0,0,0])
        self.end = True
        self.frame = np.eye(4)
        self.movenum=0

    def setSamplingtime(self, samplingTime):
        '''
        设置采样时间
        Args:
            samplingTime:

        Returns:

        '''
        self.samplingTime = samplingTime

    def setFrame(self,frame):
        '''
        设置参考坐标系
        Args:
            frame:

        Returns:

        '''
        self.frame = frame


    def setPosition(self, position):
        '''

        :param position:
        :return:
        '''

    def getPosition(self):
        '''
            获取机器臂末端位置
            Returns:

        '''

    def setVelocity(self,velocity):
        '''
        设置机械臂末端移动速度
        Args:
            velocity:

        Returns:

        '''
        self.velocity = velocity

    def run(self):
        while (self.end):
            start  = time.time()
            positionInframe = np.dot(np.linalg.inv(self.frame), self.position)
            eulerInframe = t3d.euler.mat2euler(positionInframe[:3, :3])
            nextPositionInframe = np.eye(4)
            nextPositionInframe[:3, :3] = t3d.euler.euler2mat(eulerInframe[0] - self.velocity[3] * self.samplingTime,
                                                              eulerInframe[1] - self.velocity[4] * self.samplingTime,
                                                              eulerInframe[2] - self.velocity[5] * self.samplingTime)
            nextPositionInframe[:3, 3] = positionInframe[:3, 3] - self.velocity[:3] * self.samplingTime
            self.position = np.dot(self.frame, nextPositionInframe)
            self.movenum = self.movenum + 1
            # print("nextPositionInframe",nextPositionInframe)
            # print("position",self.position)
            self.setPosition(self.position)
            # print("robotpose:",self.getPosition())
            # print("robotpose2:",self.position)
            end = time.time()
            print("one move time",end-start)
            time.sleep(self.samplingTime)

    @abstractmethod
    def realease(self):
        '''
        停止运行
        :return:
        '''
