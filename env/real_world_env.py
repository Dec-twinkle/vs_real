# _*_ coding:utf-8 _*_
# @time: 2021/9/6 上午9:14
# @author: 张新新
# @email: 1262981714@qq.com

from module.domain.kinectDK import kinectDK
from module.domain.auboRobot import auboRobot
from module.domain.aprilBoard import aprilBoard
from module.servo.pointTask import Task
from module.domain.feature import FeaturePoint
from utils import saveUtils
from utils import depthUtils
import cv2
from PIL import Image
import numpy as np
import random
import time
class env(object):
    def __init__(self):
        self.camera = kinectDK(1026,"../config/intrinsic.yml")
        self.board = aprilBoard("../config/apriltag_real.yml", "../config/tagId.csv")
        self.camera.samplingTime = 0.5
        self.camera.setSaveDir("../image")
        self.camera.show()
        self.loadRobotPose("../config/robotPoseList.json")
        self.servoTask = Task()
        self.servoTask.setHandeye(saveUtils.json_load("../config/handeye.json"))
        self.robot = auboRobot(1025)
        self.robot.setSamplingtime(0.5)



    def loadRobotPose(self,path):
        self.robotPoses = saveUtils.json_load(path)

    def reset(self):
        destRGBimage =  cv2.imread("../config/destRGBImg.bmp")
        destDepthimage =  Image.open("../config/destDepthImg.png")
        destDepthimage = np.array(destDepthimage)
        flag,objpoint_temp,imgpoint_temp = self.board.getObjImgPointList(destRGBimage)
        if not flag:
            exit(-1)
        depth_points, rejectids = depthUtils.get_depth2(imgpoint_temp, destDepthimage)
        self.destImgpoint = np.delete(imgpoint_temp, rejectids, axis=0)
        self.destDepthpoint = np.delete(depth_points, rejectids, axis=0)
        self.destobjpoint = np.delete(objpoint_temp, rejectids, axis=0)
        if self.robot.end:
            self.robot.end = False
        self.robot.setVelocity(np.array([0, 0, 0, 0, 0, 0]))
        self.robot.setPosition(self.robotPoses[0])
        time.sleep(1)
        flag, Pose = self.robot.getPosition()
        if not flag:
            exit(-1)
        self.robot.setFrame(Pose)
        #self.robot.run()

    def point2feature(self,imgpoint,depthpoint,intrinsic):
        imgpoint = imgpoint.reshape([-1,2])
        depthpoint = depthpoint.reshape([-1,1])
        feature = []
        for i in range(imgpoint.shape[0]):
            temp = np.array([imgpoint[i,0],imgpoint[i,1],1]).reshape([3,1])
            temp = np.dot(np.linalg.inv(intrinsic),temp)
            feature.append(FeaturePoint(temp[0,0],temp[1,0],depthpoint[0,0]))
        return feature


    def updateFeature(self,RGBimage,depthImage):
        flag,objpoint_temp,imgpoint_temp = self.board.getObjImgPointList(RGBimage)
        if not flag:
            print("cannot find point")
            return False
        depth_points, rejectids = depthUtils.get_depth2(imgpoint_temp, depthImage)
        sourceImgpoint = np.delete(imgpoint_temp, rejectids, axis=0)
        sourceDepthpoint = np.delete(depth_points, rejectids, axis=0)
        sourceobjpoint = np.delete(objpoint_temp, rejectids, axis=0)
        cFeatures = []
        dFeatures = []
        temp = 0
        for i in range(sourceobjpoint.shape[0]):
           for j in range(temp,self.destDepthpoint.shape[0]):
               if sourceobjpoint[i,0] ==self.destobjpoint[j,0] and sourceobjpoint[i,1] ==self.destobjpoint[j,1]:
                   temp=j
                   dFeatures.extend(self.point2feature(self.destImgpoint[j,:],self.destDepthpoint[j,:],self.camera.intrinsic))
                   cFeatures.extend(self.point2feature(sourceImgpoint[i,:],sourceDepthpoint[i,:],self.camera.intrinsic))
        self.servoTask.setCFeature(cFeatures)
        self.servoTask.setDFeature(dFeatures)
        return True

    def get_next_state(self,action):
        flag,Pose = self.robot.getPosition()
        if not flag:
            exit(-1)
        self.robot.setFrame(Pose)
        self.servoTask.setLambda(action)
        flag = self.updateFeature(self.camera.rgbImage,self.camera.depth_image)
        if not flag:
            return flag
        self.servoTask.updateFeatureError()
        vc = self.servoTask.getCameraVelocity()
        ve = self.servoTask.transformCameraToRobotVelocity(vc)
        self.robot.setVelocity(ve)
        # self.camera.get_rgb_depth_image()

        #time.sleep(self.robot.samplingTime)
        return self.robot.run()


    def getState(self,statelist):
        '''
        将当前位置转换为状态
        Args:
            position:

        Returns:

        '''
        error = self.servoTask.getError()
        n = len(self.servoTask.cFeatures)
        sum = 0
        for i in range(n):
            sum+=np.linalg.norm(error[2*i:2*i+2])
        avg = sum/n
        for state in statelist:
            if avg<state:
                return state
        return statelist[-1]

    def getState2(self, statelist):
        '''
        将当前位置转换为状态
        Args:
            position:

        Returns:

        '''
        error = self.servoTask.getError()
        n = len(self.servoTask.cFeatures)
        sum = 0
        for i in range(n):
            sum += np.linalg.norm(np.array([self.servoTask.cFeatures[i].x * self.servoTask.cFeatures[i].Z -
                                            self.servoTask.dFeatures[i].x * self.servoTask.dFeatures[i].Z,
                                            self.servoTask.cFeatures[i].y * self.servoTask.cFeatures[i].Z -
                                            self.servoTask.dFeatures[i].y * self.servoTask.dFeatures[i].Z,
                                            self.servoTask.cFeatures[i].Z - self.servoTask.dFeatures[i].Z]))
        avg = sum / n
        for state in statelist:
            if avg < state:
                return state
        return statelist[-1]

    def reward(self,threhold):
        '''

        Args:
            positon:

        Returns:

        '''


        score= 0
        acc_num = 0
        point = self.board.get_board_points()
        n = point.shape[0]
        score += -100*(n-len(self.servoTask.cFeatures))
        for i in range(len(self.servoTask.cFeatures)):
            pixel_error = np.linalg.norm(np.array([self.servoTask.error[2*i,0]*self.camera.intrinsic[0,0],
                                                   self.servoTask.error[2*i+1,0]*self.camera.intrinsic[1,1]]))
            # print(pixel_error)
            if pixel_error<threhold:
                acc_num+=1
                score+=100
            else:
                score+=-100*pixel_error/np.linalg.norm(np.array([self.camera.imgsize[0],self.camera.imgsize[1]]))
        return score/n

    def realease(self):
        self.camera.realease()
        self.robot.realease()









