from pyrobot import Robot
import numpy as np
import time
import matplotlib.pyplot as plt
base_config_dict={'base_controller': 'proportional'}
robot = Robot('locobot', base_config=base_config_dict)

print('\n---------arm test----------')
robot = Robot('locobot')
robot.arm.go_home()

start = time.time()
duration=1.0
elaptime = time.time() - start

Time=[]
JPos=[]
DPos=[]

while elaptime < (duration+1.5):
	elaptime = time.time() - start
	joint = [0,0.5*min(elaptime,duration)/duration,-1.*min(elaptime,duration)/duration,0,3.141592654]
	robot.arm.set_joint_positions(joint, plan=True, wait=False)
	current_joints = robot.arm.get_joint_angles()	
	Time.append(elaptime)
	JPos.append(current_joints)
	DPos.append(joint)

	time.sleep(0.01)

time.sleep(2)
robot.arm.go_home()

fig, axs = plt.subplots(2)

axs[0].plot(Time,[JPos[i][1] for i in range(len(JPos))])
axs[0].plot(Time,[DPos[i][1] for i in range(len(DPos))])
axs[1].plot(Time,[JPos[i][2] for i in range(len(JPos))])
axs[1].plot(Time,[DPos[i][2] for i in range(len(DPos))])
plt.show()

#print('\n---------gripper test----------')
#robot.gripper.open()
#robot.gripper.close()

#print('\n---------camera test----------\npress esc to close')
#robot.camera.reset()
#from pyrobot.utils.util import try_cv2_import
#cv2 = try_cv2_import()

#while True:
#	rgb, depth = robot.camera.get_rgb_depth()
#	cv2.imshow('Color', rgb[:, :, ::-1])
#	cv2.imshow('Depth', depth)
#	k = cv2.waitKey(30)
#	if k == 27:
#		break

#print('---------movement test----------')
#base_config_dict={'base_controller': 'proportional'}
#robot = Robot('locobot', base_config=base_config_dict)

#target_position = [1.0, 1.0, 0.0] 
#robot.base.go_to_relative(target_position, smooth=False, close_loop=True)

