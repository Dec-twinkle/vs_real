from utils import saveUtils

data = saveUtils.json_load("/home/speedbot/code/handeye_sim/result/09_09/2021_09_09_15_55/result.json")
print(data[-1])
handeye = data[-1]["Hcamera2end"]
saveUtils.json_save(handeye,"../config/handeye.json")