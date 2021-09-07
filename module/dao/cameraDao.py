# _*_ coding:utf-8 _*_
# @time: 2021/8/24 下午3:20
# @author: 张新新
# @email: 1262981714@qq.com

from abc import ABC,abstractmethod
import time
import _thread
class cameraDao(ABC):
    def __init__(self):
        self.samplingTime = 0
        self.rgbImage = None
        self.depthImage = None
        self.end = False
        self.frameNum = 0

    @abstractmethod
    def get_rgb_depth_image(self):
        '''
        获取图片
        :return:
        '''
        pass

    @abstractmethod
    def show(self):
        pass

    def run(self):
        def camera_run():
            while(not self.end):
                self.rgbImage, self.depthImage = self.get_rgb_depth_image()
                self.frameNum = self.frameNum+1
                time.sleep(self.samplingTime)
        self.end = False
        _thread.start_new_thread(camera_run, ())

    @abstractmethod
    def realease(self):
        '''
        相机停止
        :return:
        '''
        pass

