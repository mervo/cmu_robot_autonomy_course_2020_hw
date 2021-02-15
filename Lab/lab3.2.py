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
robot.arm.set_joint_positions([0., -1, 1.3, 0.92, 0.], plan = False)

#Look at floor
robot.camera.set_pan_tilt(0., 0.8, wait= True)

for i in range(10):
	#Get 3D point cloud in robot frame
	pts, rgb = robot.camera.get_current_pcd(in_cam = False)

	segPtIds=[]
	for i in range(len(pts)):
		if pts[i][0]<0.3 or pts[i][0]>0.6 or pts[i][1]>0.2 or pts[i][1]<-0.2 or pts[i][2]<-0.2:
			segPtIds.append(i)

	pts=np.delete(pts,segPtIds, 0)
	rgb=np.delete(rgb,segPtIds, 0)

	#Plot point cloud
	fig = plt.figure()
	ax = fig.add_subplot(111, projection = '3d')

	# Find highest point as grasp point
	if len(pts) > 0:
		graspPtId=np.argmax(pts[:,2])
		ax.scatter(pts[graspPtId, 0], pts[graspPtId,1], pts[graspPtId,2], color=[1,0,0])

	ax.scatter([pts[i, 0] for i in range (0, len(pts), 20)],
		[pts[i, 1] for i in range (0, len(pts), 20)],
		[pts[i, 2] for i in range (0, len(pts), 20)],
		color = [rgb[i,:]/255 for i in range(0, len(pts), 20)], marker = ".")

	plt.xlabel("X")
	plt.ylabel("Y")
	plt.pause(0.05)

	plt.show()
