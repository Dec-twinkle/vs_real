from env.real_world_env import env
import numpy as np
import time


if __name__ == '__main__':
    vs_env = env()
    vs_env.reset()
    total_time = 0.0
    sampleTime = 0.5
    while(True):
        flag = vs_env.camera.get_rgb_depth_image()
        if (not flag):
            print("cannot capture")
            break
        flag = vs_env.get_next_state(0.5)

        if not flag:
            break
        ve = vs_env.servoTask.getCameraVelocity()
        print(abs(ve[0])<0.001 and abs(ve[1])<0.001 and abs(ve[2])<0.001 and abs(ve[3])<0.001 and abs(ve[4])<0.001 and abs(ve[5])<0.001)
        if abs(ve[0])<0.001 and abs(ve[1])<0.001 and abs(ve[2])<0.001 and abs(ve[3])<0.001 and abs(ve[4])<0.001 and abs(ve[5])<0.001:
            break
        total_time+=sampleTime
        time.sleep(0.5)
        # if (np.linalg.norm(vs_env.servoTask.getCameraVelocity()) < 0.02):
        #     break

    vs_env.realease()
    print("total const time: {}(s)".format(total_time))






