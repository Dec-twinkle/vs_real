# _*_ coding:utf-8 _*_
# @time: 2021/3/24 下午2:38
# @author: 张新新
# @email: 1262981714@qq.com
# import open3d
import numpy as np
# from Module.module_base import module_base
import cv2
class FeaturePoint(obj):
    def __init__(self,x=None,y=None,Z=None):
        self.x= x
        self.y = y
        self.Z = Z

    def setXY(self,x,y):
        self.x = x
        self.y = y

    def setZ(self,Z):
        self.Z = Z


    def isVisble(self):
        return True
