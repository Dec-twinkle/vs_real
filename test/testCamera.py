# _*_ coding:utf-8 _*_
# @time: 2021/9/6 下午3:55
# @author: 张新新
# @email: 1262981714@qq.com

from module.domain.kinectDK import kinectDK
import time
if __name__ == '__main__':
    camera = kinectDK(1026,"../config/intrinsic.yml")
    camera.setSaveDir("../image")
    camera.show()
    time.sleep(10)
    camera.realease()