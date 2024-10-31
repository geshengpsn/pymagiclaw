from time import sleep
from pymagiclaw import gripper 

# 连接到gripper
g = gripper.Gripper("192.168.5.24")

# gripper标定
g.calibration()

# 模仿iphone连续发送指令
for _ in range(0, 100):
    sleep(0.033)
    g.pos(0.5)