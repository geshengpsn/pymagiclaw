from time import sleep
from pymagiclaw import franka 
import numpy as np

robot = franka.Franka("192.168.1.100", False)
robot.start_control(300, 20)

m = np.identity(4)
m[2, 3] = 0.1;

robot.move_delta_cartesian(m)
sleep(5)

robot.stop()
