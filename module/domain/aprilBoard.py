# _*_ coding:utf-8 _*_
# @time: 2021/8/26 上午10:25
# @author: 张新新
# @email: 1262981714@qq.com

import sys
ros_cv2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_cv2_path in sys.path:
    sys.path.remove(ros_cv2_path)
    import cv2
    sys.path.append(ros_cv2_path)
else:
    import cv2
import numpy as np
from module.dao.boardDao import board
import numpy as np
import cv2
import transforms3d
import dt_apriltags as apriltag
class aprilBoard(board):
    def __init__(self,configFile,tagOrderFile):
        super().__init__()
        self.conners_order = np.array([[-1, 1], [1, 1], [1, -1], [-1, -1]])
        self.getParameter(configFile)
        self.tag_id_order = np.loadtxt(tagOrderFile, delimiter=",")
        self.at_detector = apriltag.Detector(families=self.april_family)
        self.get_board_points()

    def get_board_points(self):
        self.boardcenter = np.empty([self.marker_X * self.marker_Y,2])
        self.boardcorner = np.empty([4 * self.marker_X * self.marker_Y,2])
        m,n = self.marker_X,self.marker_Y
        l = self.tag_size
        seq = self.markerSeparation
        for i in range(n):
            for j in range(m):
                center_x = j*(l+seq)
                center_y = i*(l+seq)
                self.boardcenter[i * m + j, 0] = center_x
                self.boardcenter[i * m + j, 1] = center_y
                for k in range(4):
                    self.boardcorner[4 * (i * m + j) + k, 0] = center_x + l / 2.0 * self.conners_order[k,0]
                    self.boardcorner[4 * (i * m + j) + k, 1] = center_y + l / 2.0 * self.conners_order[k,1]

    def getParameter(self, configfile):
        fs = cv2.FileStorage(configfile,cv2.FileStorage_READ)



        self.marker_X = int(fs.getNode("marker_X").real())

        self.marker_Y = int(fs.getNode("marker_Y").real())
        self.markerSeparation = fs.getNode("markerSeparation").real()
        self.tag_size = fs.getNode("tag_size").real()
        self.april_family = fs.getNode("april_family").string()
        fs.release()

    def getObjImgPointList(self,image,verbose=0):
        tags = self.detectTags(image,verbose=verbose)
        objpoint = np.array([])
        imgpoint = np.array([])
        if len(tags)<self.marker_Y*self.marker_X/2:
            return False,0,0
        for tag in tags:
            center, conners = self.getPointsbyTagId(tag.tag_id)
            objpoint = np.append(objpoint, conners)
            imgpoint = np.append(imgpoint, tag.corners)
        objpoint = np.reshape(objpoint, [-1, 2])
        imgpoint = np.reshape(imgpoint, [-1, 2])
        return True,objpoint, imgpoint

    def detectTags(self, img, cameraMatrix=None, discoff=None, verbose=0):
        """
        检测img中的apriltag，返回一组tag,如果不输入cameraMatrix，tags中不含位姿信息
        :param board: apiriltag.board 包含board的一些参数
        :param img: 需要检测的图片路径
        :param cameraMatrix:相机内参
        :return: tags 检测到的tags
        """
        # img = cv2.imread(img)
        if not discoff is None:
            img = cv2.undistort(img, cameraMatrix, discoff)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.convertScaleAbs(gray, alpha=1.5, beta=0)

        if cameraMatrix is None:

            tags = self.at_detector.detect(gray)
            cv2.waitKey(1000)

            if verbose == 1:
                img = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
                img = self.drawTagAxis(img, tags)
                cv2.namedWindow("apriltag", cv2.WINDOW_NORMAL)
                cv2.imshow("apriltag", img)
                cv2.waitKey(0)
        else:
            if not discoff is None:
                gray = cv2.undistort(gray, cameraMatrix, discoff)
            camera_param = [cameraMatrix[0, 0], cameraMatrix[1, 1], cameraMatrix[0, 2], cameraMatrix[1, 2]]
            tags = self.at_detector.detect(gray, True, camera_param, self.tag_size)
            if verbose == 1:
                img = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
                img = self.drawTagAxis(img, tags, cameraMatrix)
                cv2.namedWindow("apriltag", cv2.WINDOW_NORMAL)
                cv2.imshow("apriltag", cv2.resize(img,(512,512)))
                cv2.waitKey(0)

        return tags

    def drawTagAxis(self,img, tags, cameraMatrix=None, length=0.015, line_width=1):
        """
        在图像上画出每个tag的坐标轴，蓝色表示x轴，绿色表示y轴，红色表示z轴
        :param img: 图片
        :param tags:
        :param cameraMatrix: 相机内参
        :param length: 长度，指实际长度
        :return: img：图片
        """
        point_x = np.array([[length, 0, 0, 1]]).T
        point_y = np.array([[0, length, 0, 1]]).T
        point_z = np.array([[0, 0, length, 1]]).T
        if cameraMatrix is None:
            for tag in tags:
                img = cv2.circle(img, (int(tag.corners[0, 0]), int(tag.corners[0, 1])), 5, (255, 0, 0), thickness=3)
                img = cv2.circle(img, (int(tag.corners[1, 0]), int(tag.corners[1, 1])), 5, (0, 255, 0), thickness=3)
                img = cv2.circle(img, (int(tag.corners[2, 0]), int(tag.corners[2, 1])), 5, (0, 0, 255), thickness=3)
                cv2.putText(img, str(tag.tag_id), (int(tag.center[0]), int(tag.center[1])), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (0, 255, 0), 2)
            return img
        cameraMatrix = np.append(cameraMatrix, np.zeros([3, 1]), 1)
        for tag in tags:
            R = tag.pose_R
            T = tag.pose_t
            H = np.append(np.append(R, T, 1), np.array([[0, 0, 0, 1]]), 0)
            pro_x = np.dot(cameraMatrix, np.dot(H, point_x))
            pro_x = pro_x / pro_x[2, 0]
            pro_y = np.dot(cameraMatrix, np.dot(H, point_y))
            pro_y = pro_y / pro_y[2, 0]
            pro_z = np.dot(cameraMatrix, np.dot(H, point_z))
            pro_z = pro_z / pro_z[2, 0]

            cv2.putText(img, str(tag.tag_id), (int(tag.center[0]), int(tag.center[1])), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 0), 2)
            img = cv2.line(img, (int(tag.center[0]), int(tag.center[1])), (int(pro_x[0, 0]), int(pro_x[1, 0])),
                           (255, 0, 0), thickness=line_width)
            img = cv2.line(img, (int(tag.center[0]), int(tag.center[1])), (int(pro_y[0, 0]), int(pro_y[1, 0])),
                           (0, 255, 0), thickness=line_width)
            img = cv2.line(img, (int(tag.center[0]), int(tag.center[1])), (int(pro_z[0, 0]), int(pro_z[1, 0])),
                           (0, 0, 255), thickness=line_width)
            # img = cv2.circle(img,(int(tag.corners[0,0]),int(tag.corners[0,1])),1,(255,0,0),thickness=2)
            # img = cv2.circle(img,(int(tag.corners[1,0]),int(tag.corners[1,1])),1,(0,255,0),thickness=2)
            # img = cv2.circle(img,(int(tag.corners[2,0]),int(tag.corners[2,1])),1,(0,0,255),thickness=2)

        return img

    def getPointsbyTagId(self,tagId):
        x,y = np.where(self.tag_id_order==tagId)
        center = self.boardcenter[x[0]*self.marker_X+y[0],:]
        corner = self.boardcorner[4*(x[0]*self.marker_X+y[0]):4*(x[0]*self.marker_X+y[0])+4,:]
        return center,corner