# _*_ coding:utf-8 _*_
# @time: 2021/8/24 下午2:59
# @author: 张新新
# @email: 1262981714@qq.com

from abc import ABC,abstractmethod
import numpy as np
import time
import transforms3d as t3d
import _thread

class robotDao(ABC):

    def __init__(self):
        self.samplingTime = 0
        self.position = None
        self.velocity = np.array([0,0,0,0,0,0])
        self.end = True
        self.frame = np.eye(4)

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

    @abstractmethod
    def setPosition(self, position):
        '''

        :param position:
        :return:
        '''
    @abstractmethod
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
        def robot_run():
            while(self.end):
                positionInframe = np.dot(np.linalg.inv(self.frame),self.position)
                eulerInframe = t3d.euler.mat2euler(positionInframe[:3,:3])
                nextPositionInframe = np.eye(4)
                nextPositionInframe[:3,:3] = t3d.euler.euler2mat(eulerInframe[0]-self.velocity[3]*self.samplingTime,
                                                                 eulerInframe[1]-self.velocity[4]*self.samplingTime,
                                                                 eulerInframe[2]-self.velocity[5]*self.samplingTime)
                nextPositionInframe[:3,3] = positionInframe[:3,3]-self.velocity[:3] * self.samplingTime
                self.position = np.dot(self.frame,nextPositionInframe)
                # print("robotpose:",self.getPosition())
                time.sleep(self.samplingTime)
        self.end = True
        _thread.start_new_thread(robot_run, ())

    @abstractmethod
    def end(self):
        '''
        停止运行
        :return:
        '''
