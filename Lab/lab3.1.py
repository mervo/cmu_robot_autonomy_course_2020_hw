from pyrobot import Robot
import numpy as np 
import time
import matplotlib.pyplot as plt

robot = Robot('locobot')

robot.camera.reset()
from pyrobot.utils.util import try_cv2_import

cv2 = try_cv2_import()
robot.camera.depth_cam.cfg_data['DepthMapFactor'] = 1.0

while True:
	rgb, depth = robot.camera.get_rgb_depth()
	cv2.imshow('Color', rgb[:, :, ::-1])
	cv2.imshow('Depth', depth)
	if cv2.waitKey(1) & 0xFF == ord('q'):
	      break
