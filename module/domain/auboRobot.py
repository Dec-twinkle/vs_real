# _*_ coding:utf-8 _*_
# @time: 2021/8/24 下午3:13
# @author: 张新新
# @email: 1262981714@qq.com

# from module.dao import robotDao
from socket import socket
import transforms3d as t3d
import _thread
import numpy as np
import time
class auboRobot():
    def __init__(self, port):
        super().__init__()
        self.port = port
        self.socket = socket()
        self.socket.connect(("127.0.0.1", self.port))
        self.samplingTime = 0
        self.position = None
        self.velocity = np.array([0, 0, 0, 0, 0, 0])
        self.end = False
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

    def setVelocity(self,velocity):
        '''
        设置机械臂末端移动速度
        Args:
            velocity:

        Returns:

        '''
        self.velocity = velocity

    def endrun(self):
        self.end = True

    def getPosition(self):
        time.sleep(0.1)
        order = "cur"
        self.socket.send(order.encode())
        recv = self.socket.recv(1024).decode()
        if recv == "False" or recv == '':
            return False, None
        pose_str_list = recv.split(',')
        flag = pose_str_list[0]
        t1 = np.array([float(pose_str_list[1]), float(pose_str_list[2]), float(pose_str_list[3])])
        q1 = np.array(
            [float(pose_str_list[4]), float(pose_str_list[5]), float(pose_str_list[6]), float(pose_str_list[7])])
        r = t3d.quaternions.quat2mat(q1)
        pose = np.identity(4)
        pose[:3, :3] = r[:, :]
        pose[:3, 3] = t1[:]
        self.position = pose
        # time.sleep(5)
        return True, pose

    def run(self):
        print("ve", self.velocity)
        positionInframe = np.dot(np.linalg.inv(self.frame), self.position)
        eulerInframe = t3d.euler.mat2euler(positionInframe[:3, :3])
        nextPositionInframe = np.eye(4)
        nextPositionInframe[:3, :3] = t3d.euler.euler2mat(eulerInframe[0] - self.velocity[3] * self.samplingTime,
                                                          eulerInframe[1] - self.velocity[4] * self.samplingTime,
                                                          eulerInframe[2] - self.velocity[5] * self.samplingTime)
        nextPositionInframe[:3, 3] = positionInframe[:3, 3] - self.velocity[:3] * self.samplingTime
        self.position = np.dot(self.frame, nextPositionInframe)
        # print("position:", self.position)
#        self.movenum = self.movenum + 1
        flag = self.setPosition(self.position)
        return flag

        # def robot_run():
        #     while(not self.end):
        #         #print("robot is running!!")
        #         # print("frame:", self.frame)
        #         print("ve",self.velocity)
        #         positionInframe = np.dot(np.linalg.inv(self.frame),self.position)
        #         eulerInframe = t3d.euler.mat2euler(positionInframe[:3,:3])
        #         nextPositionInframe = np.eye(4)
        #         nextPositionInframe[:3,:3] = t3d.euler.euler2mat(eulerInframe[0]-self.velocity[3]*self.samplingTime,
        #                                                          eulerInframe[1]-self.velocity[4]*self.samplingTime,
        #                                                          eulerInframe[2]-self.velocity[5]*self.samplingTime)
        #         nextPositionInframe[:3,3] = positionInframe[:3,3]-self.velocity[:3] * self.samplingTime
        #         self.position = np.dot(self.frame,nextPositionInframe)
        #         # print("position:", self.position)
        #         self.movenum = self.movenum+1
        #         self.setPosition(self.position)
        #         time.sleep(10)
        #         # print("robot stop running!!")
        # self.end = False
        # self.movenum=0
        # _thread.start_new_thread(robot_run, ())



    def setPosition(self, position):
        #print("set position!!!!")
        order = "move"
        time.sleep(0.5)
        self.socket.send(order.encode())
        q = t3d.quaternions.mat2quat(position[:3, :3])
        t = position[:3, 3]
        self.socket.send("{},{},{},{},{},{},{}".format(t[0], t[1], t[2], q[0], q[1], q[2], q[3]).encode())
        # print("send", q, t)
        recv = self.socket.recv(1024).decode()
        # print("recive ", recv)


        if recv == "False":
            return False#, None

        # pose_str_list = recv.split(',')
        # flag = pose_str_list[0]
        # t1 = np.array([float(pose_str_list[1]), float(pose_str_list[2]), float(pose_str_list[3])])
        # q1 = np.array(
        #     [float(pose_str_list[4]), float(pose_str_list[5]), float(pose_str_list[6]), float(pose_str_list[7])])
        # r = t3d.quaternions.quat2mat(q1)
        # pose = np.identity(4)
        # pose[:3, :3] = r[:, :]
        # pose[:3, 3] = t1[:]
        # time.sleep(5)
        return True#, pose

    def realease(self):
        order = "end"
        self.socket.send(order.encode())
        self.socket.close()



