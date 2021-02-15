from pyrobot import Robot
import numpy as np 
import time
import matplotlib.pyplot as plt

robot = Robot('locobot')

robot.camera.reset()
from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()
robot.camera.depth_cam.cfg_data['DepthMapFactor'] = 1.0

#Move robot out of vision area
robot.arm.go_home()
robot.arm.set_joint_positions([0., -1, 1.3, 0.92, 0.], plan = False)

#Look at floor
robot.camera.set_pan_tilt(0., 0.8, wait= True)

while True:
	rgb, depth = robot.camera.get_rgb_depth()
	cv2.imshow('Color', rgb[:, :, ::-1])
	cv2.imshow('Depth', depth)
	if cv2.waitKey(1) & 0xFF == ord('q'):
	      break
