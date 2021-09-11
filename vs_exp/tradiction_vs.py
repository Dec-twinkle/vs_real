from env.real_world_env import env
import numpy as np
import time


if __name__ == '__main__':
    vs_env = env()
    vs_env.reset()
    while(True):
        flag = vs_env.camera.get_rgb_depth_image()
        if(not flag):
            print("cannot capture")
            break
        vs_env.get_next_state(0.5)
        if(np.linalg.norm(vs_env.servoTask.getCameraVelocity())<0.02):
            break
    vs_env.realease()






