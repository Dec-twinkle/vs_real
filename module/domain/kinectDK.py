# _*_ coding:utf-8 _*_
# @time: 2021/8/24 下午3:35
# @author: 张新新
# @email: 1262981714@qq.com

from module.dao import cameraDao
from socket import socket
from PIL import Image
import numpy as np
import sys
import os
import time
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
import _thread
class kinectDK(cameraDao.cameraDao):
    def __init__(self, port, config):
        super().__init__()
        self.port = port
        self.socket = socket()
        self.temp_color_file_name = "../buffer/rgb.bmp"
        self.temp_depth_file_name = "../buffer/depth.png"
        self.imgsize = (1920, 1080)
        fs2 = cv2.FileStorage(config, cv2.FileStorage_READ)
        self.intrinsic = fs2.getNode("intrinsic").mat()
        self.dist = fs2.getNode("dist").mat()
        fs2.release()
        self.socket.connect(("127.0.0.1", self.port))
        self.savedir = None
        self.flag = True
        self.end = False

    def get_rgb_depth_image(self):
        try:
            self.socket.send("capture".encode())
            flag = self.socket.recv(1024).decode()
            if flag == 'True':
                self.rgbImage = cv2.imread(self.temp_color_file_name)
                #os.remove(self.temp_color_file_name)
                depth_image = Image.open(self.temp_depth_file_name)
                self.depth_image = np.array(depth_image)
                return True
                #os.remove(self.temp_depth_file_name)
                # img = Image.fromarray(depth_image)
                # img.save("../temp/test_save_depth.png")
                # return True, image, depth_image
        except Exception:
            print("fail to connect")
            self.socket.close()
            self.end = True
            self.flag = False
            return False
            # return False,None,None

    def setSaveDir(self,dir):
        self.savedir = dir

    def show(self):
        def cv2show():
            while(not self.end):

                # flag = self.get_rgb_depth_image()
                # if not flag:
                #     print("fail to capture")
                #     exit(-1)
                if self.rgbImage is None:
                    time.sleep(self.samplingTime)
                else:
                    cv2.imshow("kinect capture", cv2.resize(self.rgbImage,(192*4,108*4)))
                    cv2.waitKey(int(self.samplingTime*1000))
                    if not self.savedir is None:
                        savepath = os.path.join(self.savedir,"{}.png".format(str(self.frameNum).zfill(4)))
                        if not os.path.exists(savepath):
                            cv2.imwrite(savepath,self.rgbImage)

        _thread.start_new_thread(cv2show, ())

    def realease(self):
        self.socket.send("end".encode())
        self.socket.close()
        return





