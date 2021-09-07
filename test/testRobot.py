# _*_ coding:utf-8 _*_
# @time: 2021/9/6 下午4:01
# @author: 张新新
# @email: 1262981714@qq.com
from module.domain.auboRobot import auboRobot
import numpy as np
import time
if __name__ == '__main__':
    robot = auboRobot(1025)
    robot.setFrame(robot.getPosition())
    robot.setSamplingtime(0.1)
    robot.setVelocity(np.array([0,0,0,0.05,0.05,0.05]))
    robot.run()
    time.sleep(10)
    robot.end()