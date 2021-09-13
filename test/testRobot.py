# _*_ coding:utf-8 _*_
# @time: 2021/9/6 下午4:01
# @author: 张新新
# @email: 1262981714@qq.com
from module.domain.auboRobot import auboRobot
import os
from utils import saveUtils
import numpy as np
import time
import threading
if __name__ == '__main__':
    robot = auboRobot(1025)
    if os.path.exists("../config/robotPoseList.json"):
        pose_list = saveUtils.json_load("../config/robotPoseList.json")
    else:
        pose_list = []
    while(True):
        flag,position = robot.getPosition()
        pose_list.append(position)
        a = input()
        if a=="0":
            break
    robot.realease()
    saveUtils.json_save(pose_list,"../config/robotPoseList.json")
    # robot.setFrame(position)
    # robot.setSamplingtime(0.5)
    # robot.setVelocity(np.array([0,0,0.02,0.0,0,0.0]))
    # thread_pool = []
    # thread_pool.append(robot)
    # for th in thread_pool:
    #     th.start()
    # time.sleep(10)
    # robot.realease()
    # time.sleep(20)
