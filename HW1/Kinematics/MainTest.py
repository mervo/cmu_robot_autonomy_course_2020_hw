import Locobot
import numpy as np

#Initialize robot object
mybot=Locobot.Locobot()

# Compute forward kinematics
deg_to_rad = np.pi/180.

joint_targets = [[  0.,
                        0.,
                        0.,
                        0.,
                        0.], \
                     [-45.*deg_to_rad,
                      -15.*deg_to_rad,
                       20.*deg_to_rad,
                       15.*deg_to_rad,
                      -75.*deg_to_rad], \
                     [ 30.*deg_to_rad,
                       60.*deg_to_rad,
                      -65.*deg_to_rad,
                       45.*deg_to_rad,
                        0.*deg_to_rad]]

for joint_target in joint_targets:
  Hcurr,J =mybot.ForwardKin(joint_target)
  ee_pose = Hcurr[-1]
  rot_ee = ee_pose[:3,:3]
  pos_ee = ee_pose[:3,3]
  print('computed FK ee position', pos_ee)
  print('computed FK ee rotation', rot_ee)

# Compute inverse kinematics
qInit=[0.0,-0.5,1.0,0.5,0.5] # initial joint positions
HGoal= np.array([[1,0,0,0.5], # target EE pose
		 [0,1,0,0.0],
		 [0,0,1,0.25],
		 [0,0,0,1]])

q,Err=mybot.IterInvKin(qInit, HGoal)
print('error', Err)
print('compute IK angles', q)
# mybot.PlotSkeleton(q)
