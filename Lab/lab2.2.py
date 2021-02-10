from pyrobot import Robot
import numpy as np 
import time
import matplotlib.pyplot as plt
import pickle
import math

f = open("myDemo.p", 'rb')
demoTime = pickle.load(f)
demoJoint = pickle.load(f)
f.close()

robot = Robot('locobot')
robot.arm.go_home()

robot.arm.set_joint_positions(demoJoint[0], plan=True)
time.sleep(2)

start = time.time()
duration = demoTime[-1]
elaptime = time.time()-start

w = len(demoTime)*[0]
wsum = 0.0

Time=[]
JPos = []
DPos = []

while elaptime < (duration + 0.05):
	elaptime = time.time() - start

	wsum = 0.0
	for i in range (len(w)):
		w[i] = math.exp(-0.5*(elaptime-demoTime[i])**2/0.05**2)
		wsum+=w[i]

	joint = [0.0,0.0,0.0,0.0,0.0]
	for i in range(len(w)):
		for j in range (len(joint)):
			joint[j]+=demoJoint[i][j]*w[i]/wsum

	robot.arm.set_joint_positions(joint, plan=False, wait=False)

	current_joints = robot.arm.get_joint_angles()

	Time.append(elaptime)
	JPos.append(current_joints)
	DPos.append(joint)

	time.sleep(0.01)

time.sleep(2)

robot.arm.go_home()