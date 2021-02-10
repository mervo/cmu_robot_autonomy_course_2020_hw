from pyrobot import Robot
import numpy as np 
import time
import matplotlib.pyplot as plt
import pickle

arm_config = dict(control_mode='torque')
robot = Robot('locobot', arm_config = arm_config)

target_torque = 4*[0]
robot.arm.set_joint_torques(target_torque)
robot.arm.go_home()

duration = 10.0
Time = []
JPos = []
print("Go to start position")
# time.sleep(5)
# print("3...")
# time.sleep(1)
# print("2...")
# time.sleep(1)
# print("1...")
# time.sleep(1)
print("Starting to record")

start = time.time()
elaptime = time.time() - start

while elaptime < (duration+0.5):
	elaptime = time.time() - start
	current_joints = robot.arm.get_joint_angles()

	Time.append(elaptime)
	JPos.append(current_joints)

	time.sleep(0.01)

print("Recording ended")

# target_torque = 4*[1]
# robot.arm.set_joint_torques(target_torque)
# robot.arm.go_home()

fig, axs = plt.subplots(2)

axs[0].plot(Time,[JPos[i][1] for i in range(len(JPos))])
axs[1].plot(Time,[JPos[i][2] for i in range(len(JPos))])
print(JPos)
plt.show()

f = open("myDemo.p", 'wb')
pickle.dump(Time, f)
pickle.dump(JPos, f)
f.close()