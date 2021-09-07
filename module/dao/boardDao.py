# _*_ coding:utf-8 _*_
# @time: 2021/8/26 上午9:37
# @author: 张新新
# @email: 1262981714@qq.com

from abc import ABCMeta,abstractmethod
import numpy as np
import transforms3d
from scipy import optimize as op
import sys
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
from matplotlib import pyplot as plt
from module.utils import plane_utils
class board(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def getParameter(self,configfile):
        pass


    @abstractmethod
    def GetBoardAllPoints(self):
        """
        获取标定板上所有可计算的角点
        :return: objpoints: 所有角点
        """
        pass

    @abstractmethod
    def getObjImgPointList(self,image,verbose=0):
        '''
        提取tags中的角点，board的坐标
        :param tags: apriltag检测的tag
        :param board: apriltag板，内含apriltag的一些参数
        :return: objPoint board对应角点
                imgPoint 对应角点在图片上的坐标
        '''
        pass

    def getPointDepth(self,imgPoints, depthImage):
        """
        获取对应点的深度
        :param imgPoints: rgb图片上的角点
        :param depthImage: 深度图片
        :return:
        """
        depth_point = np.array([])
        img_point = np.array([])
        for j in range(imgPoints.shape[0]):
            depth = 0
            try:
                depth = depthImage[int(imgPoints[j, 1]), int(imgPoints[j, 0])] / 1000.0
            except Exception:
                depth = 0
            if depth > 0.2:
                depth_point = np.append(depth_point,depth)
                img_point = np.append(img_point,imgPoints[j,:])
        return np.shape(img_point,(-1,2)),np.shape(depth_point,(-1,1))

    def imgPointDepthOpt(self,imgPoints,depthPoints):
        """
        通过平面拟合的方式，去除深度图像中的噪声
        :param imgPoints:
        :param depthPoints:
        :return:
        """
        point_in_camera = np.append(imgPoints, np.ones([imgPoints.shape[0], 1]), 1)
        point_in_camera[:, :] = np.multiply(point_in_camera, np.repeat(depthPoints, 3, axis=1))
        plane = plane_utils.get_nice_plane(point_in_camera)
        for j in range(point_in_camera.shape[0]):
            depthPoints[j, 0] = plane[0] * point_in_camera[j, 0] + plane[1] * point_in_camera[j, 1] + plane[2]
        return depthPoints



