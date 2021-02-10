from pyrobot import Robot
import numpy as np
import time
import matplotlib.pyplot as plt

robot = Robot('locobot')

robot.arm.go_home()

position=np.array([0.325,0.0,0.3])
orientation=np.array([[0.0,0.0,1.0],[0.0,1.0,0.0],[-1.0,0.0,0.0]])
robot.arm.set_ee_pose(position,orientation,plan=True,wait=True,numerical=True)
time.sleep(1)

Time=[]
EPos=[]
EOri=[]
DPos=[]
DOri=[]

start = time.time()
duration=3.0
elaptime = time.time() - start

while elaptime < (duration+1.5):
	elaptime = time.time() - start
	position=np.array([0.325+0.1*min(elaptime,duration)/duration,0.0,0.3-0.1*min(elaptime,duration)/duration])
	orientation=np.array([[0.0,0.0,1.0],[0.0,1.0,0.0],[-1.0,0.0,0.0]])
	robot.arm.set_ee_pose(position,orientation,plan=False,wait=True, numerical=True)


	current_joints = robot.arm.get_joint_angles()
	myEEpose=robot.arm.pose_ee

	Time.append(elaptime)
	EPos.append(myEEpose[0])
	EOri.append(myEEpose[1])
	DPos.append(position)
	DOri.append(orientation)
	# time.sleep(0.01)

time.sleep(2)

robot.arm.go_home()

fig, axs = plt.subplots(2)

axs[0].plot(Time,[EPos[i][0] for i in range(len(EPos))])
axs[0].plot(Time,[DPos[i][0] for i in range(len(DPos))])
axs[1].plot(Time,[EPos[i][2] for i in range(len(EPos))])
axs[1].plot(Time,[DPos[i][2] for i in range(len(DPos))])
plt.show()