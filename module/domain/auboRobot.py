# _*_ coding:utf-8 _*_
# @time: 2021/8/24 下午3:13
# @author: 张新新
# @email: 1262981714@qq.com

from module.dao import robotDao
from socket import socket
import transforms3d
import numpy as np
import time
class auboRobot(robotDao.robotDao):
    def __init__(self, port):
        super().__init__()
        self.port = port
        self.socket = socket()
        self.socket.connect(("127.0.0.1", self.port))

    def getPosition(self):
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
        r = transforms3d.quaternions.quat2mat(q1)
        pose = np.identity(4)
        pose[:3, :3] = r[:, :]
        pose[:3, 3] = t1[:]
        # time.sleep(5)
        return True, pose

    def setPosition(self, position):
        while (True):
            try:
                order = "move"
                self.socket.send(order.encode())
                q = transforms3d.quaternions.mat2quat(position[:3, :3])
                t = position[:3, 3]
                self.socket.send("{},{},{},{},{},{},{}".format(t[0], t[1], t[2], q[0], q[1], q[2], q[3]).encode())
                recv = self.socket.recv(1024).decode()
                break
            except Exception:
                a = input("please restart robot and input")
                self.socket.close()
                self.socket = socket()
                self.socket.connect(("127.0.0.1", self.port))

        if recv == "False" or recv == "":
            return False, None

        pose_str_list = recv.split(',')
        flag = pose_str_list[0]
        t1 = np.array([float(pose_str_list[1]), float(pose_str_list[2]), float(pose_str_list[3])])
        q1 = np.array(
            [float(pose_str_list[4]), float(pose_str_list[5]), float(pose_str_list[6]), float(pose_str_list[7])])
        r = transforms3d.quaternions.quat2mat(q1)
        pose = np.identity(4)
        pose[:3, :3] = r[:, :]
        pose[:3, 3] = t1[:]
        # time.sleep(5)
        return True, pose

    def end(self):
        order = "end"
        self.socket.send(order.encode())
        self.socket.close()

