from time import sleep
from pymagiclaw import franka 
import numpy as np

# 输入ip，系统是否有实时内核
robot = franka.Franka("192.168.1.100", False)

# tranlation stiffness, rotation stiffness
robot.start_control(300, 20)

# 绝对坐标
robot.move_absolute_cartesian(
    np.array(
        [[ 0.92342271, -0.35936834,  0.13470301,  0.48263934],
         [-0.36170872, -0.93226071, -0.00753466, -0.03020527],
        [ 0.12828604, -0.04176558, -0.99085737,  0.22651136],
        [ 0.,          0.,          0.,          1.        ]]
    )
);
sleep(1)

# 相对坐标
m = np.identity(4)
m[2, 3] = 0.1;
robot.move_relative_cartesian(m)
sleep(1)

# 末端位姿
state = robot.read_state()
print(state)

# sleep(10)

# robot.stop()
