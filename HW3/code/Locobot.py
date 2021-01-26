import numpy as np
import math
import time
import random

import RobotUtil as rt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Locobot:

    def __init__(self):

        # Robot descriptor from URDF file (rpy xyz for each link)
        self.Rdesc = [
            [0, 0, 0, 0.0973, 0, 0.1730625],  # From robot base to joint1
            [0, 0, 0, 0, 0, 0.04125],
            [0, 0, 0, 0.05, 0, 0.2],
            [0, 0, 0, 0.2002, 0, 0],
            [0, 0, 0, 0.063, 0.0001, 0],
            [0, 0, 0, 0.106525, 0, 0.0050143]  # From joint5 to end-effector center
        ]

        # Define the axis of rotation for each joint - use your self.axis from Homework 1
        '''Note: you will get this from the URDF (interbotix_locobot_description.urdf).
        For example, in the urdf under each joint you will see: (<axis xyz="0 1 0"/>)
        '''
        self.axis = ([[0, 0, 1],
                      [0, 1, 0],
                      [0, 1, 0],
                      [0, 1, 0],
                      [-1, 0, 0],
                      [0, 1, 0]
                      ])

        # joint limits for arm
        self.qmin = [-1.57, -1.57, -1.57, -1.57, -1.57]
        self.qmax = [1.57, 1.57, 1.57, 1.57, 1.57]

        # Robot collision blocks descriptor (base frame, (rpy xyz), length/width/height
        # NOTE: Cidx and Cdesc are just the robot's link BB's
        self.Cidx = [1, 2, 3, 4]  # which joint frame the BB should be defined in

        # xyz rpy poses of the robot arm blocks (use to create transforms)

        self.Cdesc = [[0., 0., 0., 0., 0., 0.09],
                      [0., 0., 0., 0.075, 0., 0.],
                      [0., 0., 0., 0.027, -0.012, 0.],
                      [0., 0., 0., 0.055, 0.0, 0.01],
                      ]

        # dimensions of robot arm blocks
        self.Cdim = [[0.05, 0.05, 0.25],
                     [0.25, 0.05, 0.05],
                     [0.07, 0.076, 0.05],
                     [0.11, 0.11, 0.07],
                     ]

        # Set base coordinate frame as identity
        self.Tbase = [[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]]

        # Initialize matrices
        self.Tlink = []  # Transforms for each link (const)
        self.Tjoint = []  # Transforms for each joint (init eye)
        self.Tcurr = []  # Coordinate frame of current (init eye)
        for i in range(len(self.Rdesc)):
            self.Tlink.append(rt.rpyxyz2H(self.Rdesc[i][0:3], self.Rdesc[i][3:6]))
            self.Tcurr.append([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.], [0, 0, 0, 1]])
            self.Tjoint.append([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.], [0, 0, 0, 1]])

        self.Tlink[0] = np.matmul(self.Tbase, self.Tlink[0])

        self.J = np.zeros((6, 5))

        self.q = [0., 0., 0., 0., 0., 0.]
        self.ForwardKin([0., 0., 0., 0., 0.])

        self.Tblock = []  # Transforms for each arm block
        self.Tcoll = []  # Coordinate frame of current collision block

        self.Cpoints = []
        self.Caxes = []

        for i in range(len(self.Cdesc)):
            self.Tblock.append(rt.rpyxyz2H(self.Cdesc[i][0:3], self.Cdesc[i][3:6]))
            self.Tcoll.append([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.], [0, 0, 0, 1]])

            self.Cpoints.append(np.zeros((3, 4)))
            self.Caxes.append(np.zeros((3, 3)))

    def ForwardKin(self, ang):
        '''
        inputs: joint angles
        outputs: joint transforms for each joint, Jacobian matrix
        '''
        self.q[0:-1] = ang
        # TODO: implement your Forward Kinematics here
        # TODO: Compute current joint and end effector coordinate frames (self.Tjoint). Remember that not all joints rotate about the z axis!

        for i in range(len(self.q)):
            self.Tjoint[i] = rt.rpyxyz2H(np.array(self.axis[i]) * self.q[i], [0, 0, 0])

            # z-axis only example
            # self.Tjoint[i] = [[math.cos(self.q[i]), -math.sin(self.q[i]), 0, 0],
            # 				  [math.sin(self.q[i]), math.cos(self.q[i]), 0, 0],
            # 				  [0, 0, 1, 0],
            # 				  [0, 0, 0, 1]]

            if i == 0:
                self.Tcurr[i] = np.matmul(self.Tlink[i], self.Tjoint[i])
            else:
                self.Tcurr[i] = np.matmul(np.matmul(self.Tcurr[i - 1], self.Tlink[i]), self.Tjoint[i])

        # TODO: Compute Jacobian matrix
        for i in range(len(self.Tcurr) - 1):
            # Position of end effector - Position of ith joint
            p = self.Tcurr[-1][0:3, 3] - self.Tcurr[i][0:3, 3]  # From all 3 elements, return index 3
            axis = np.argwhere(self.axis[i])[0][0]  # Define axis to use as joint axis
            a = self.Tcurr[i][0:3, axis]  # Lecture, about xx axis
            self.J[0:3, i] = np.cross(a, p)
            self.J[3:7, i] = a
        return self.Tcurr, self.J

    def IterInvKin(self, ang, TGoal):
        '''
        inputs: starting joint angles (ang), target end effector pose (TGoal)

        outputs: computed joint angles to achieve desired end effector pose,
        Error in your IK solution compared to the desired target
        '''
        self.ForwardKin(ang)

        Err = [0, 0, 0, 0, 0, 0]  # error in position and orientation, initialized to 0
        for s in range(1000):
            # print(f'Step: {s}')
            # TODO: Compute rotation error (radian)
            rErrR = np.matmul(TGoal[0:3, 0:3], np.transpose(self.Tcurr[-1][0:3, 0:3]))  # R_err = R_goal * RT_ECurr
            rErrAxis, rErrAng = rt.R2axisang(rErrR)  # Convert to axis angle form
            # print(f'Pos Error(m): {rErrAxis}, Rotation Error(rad): {rErrAng}')

            # Limit rotation angle
            if rErrAng > 0.1:  # 0.1 rad ~ 5.7 deg
                rErrAng = 0.1 * np.pi / 180  # 0.1 deg
            if rErrAng < -0.1:
                rErrAng = - 0.1 * np.pi / 180

            rErr = [rErrAxis[0] * rErrAng, rErrAxis[1] * rErrAng, rErrAxis[2] * rErrAng]

            # TODO: Compute position error (meter)
            # Compute and limit translation error
            xErr = TGoal[0:3, 3] - self.Tcurr[-1][0:3, 3]
            if np.linalg.norm(xErr) > 0.01:
                xErr = xErr * 0.01 / np.linalg.norm(xErr)

            Err[0:3] = xErr
            Err[3:6] = rErr
            # TODO: Update joint angles with Jacobian Pseudo-Inverse Approach J^T(JJ^T)^-1 S43
            # Jacobian Pseudo-Inverse cannot be computed @ Step 4 even after reducing step sizes multiple times
            # Pos Error(m): [0.41623425993423047, -0.8802044262027227, -0.22801142285012657], Rotation Error(rad): 1.1072814432454838
            # self.q[0:-1] = self.q[0:-1] + np.matmul(
            #     np.matmul(np.transpose(self.J), np.linalg.inv(np.matmul(self.J, np.transpose(self.J)))), Err)

            # Switching to Jacobian Transpose Method

            # TODO: Update joint angles with Jacobian Transpose Approach alpha * J^T S33
            # Step: 1000
            # Pos Error(m): [0.7712178398298731, -0.6167892348244101, -0.15746137092272164], Rotation Error(rad): 0.767704823590883
            # computed IK angles[-0.02715018929923225, -0.5629399578355448, 0.7531900187871879, 0.290465299712724, 0.610573795437234]
            # Step: 2000
            # Pos Error(m): [0.8471681122638188, -0.507206807657038, -0.1582638424584537], Rotation Error(rad): 0.8643796036834195
            # computed IK angles [-0.03371279976628066, -0.45473591137218317, 0.7042210161901095, 0.1981941678980578, 0.7528826325651012]
            self.q[0:-1] = self.q[0:-1] + 0.1 * np.matmul(np.transpose(self.J), Err)

            # TODO: Recompute forward kinematics(and loop) for new angles
            self.ForwardKin(self.q[0:-1])

        return self.q[0:-1], Err

    def SampleRobotConfig(self):
        # implement random sampling of robot joint configurations
        q = []

        assert len(self.qmax) == len(self.qmin)

        for i in range(len(self.qmax)):
            q_1 = np.random.uniform(self.qmin[i], self.qmax[i])
            q.append(q_1)

        return q

    def CompCollisionBlockPoints(self, ang):
        '''
        Get robot's collision boxes
        :param ang:
        :return:
        '''
        # Use your FK implementation here to compute collision boxes for the robot arm
        self.ForwardKin(ang)

        # Compute current collision boxes for arm
        for i in range(len(self.Cdesc)):
            self.Tcoll[i] = np.matmul(self.Tcurr[self.Cidx[i]], self.Tblock[i])  # Joint frame and local box transform
            self.Cpoints[i], self.Caxes[i] = rt.BlockDesc2Points(self.Tcoll[i], self.Cdim[i])

    def DetectCollision(self, ang, pointsObs, axesObs):
        # implement collision detection using CompCollisionBlockPoints() and rt.CheckBoxBoxCollision()
        self.CompCollisionBlockPoints(ang)

        for i in range(len(self.Cpoints)):
            for j in range(len(pointsObs)):
                if rt.CheckBoxBoxCollision(self.Cpoints[i], self.Caxes[i], pointsObs[j], axesObs[j]):
                    return True

        return False

    def DetectCollisionEdge(self, ang1, ang2, pointsObs, axesObs):
        # Detects if an edge is valid or in collision
        for s in np.linspace(0, 1, 50):
            ang = [ang1[k] + s * (ang2[k] - ang1[k]) for k in range(len(ang1))]

            if self.DetectCollision(ang, pointsObs, axesObs):
                return True

        return False

    def RobotInLimits(self, ang):
        # Use this to check if a joint configuration is within robot's joint limits
        for i in range(5):
            if ang[i] < self.qmin[i] or ang[i] > self.qmax[i]:
                return False
        return True

    def PlotCollisionBlockPoints(self, ang, pointsObs=[]):
        # This is a plotting function to visualize you robot with obstacles

        # Compute collision block points
        self.CompCollisionBlockPoints(ang)
        # Create figure
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Draw links along coordinate frames
        for i in range(len(self.Tcurr)):
            ax.scatter(self.Tcurr[i][0, 3], self.Tcurr[i][1, 3], self.Tcurr[i][2, 3], c='k', marker='.')
            if i is 0:
                ax.plot([0, self.Tcurr[i][0, 3]], [0, self.Tcurr[i][1, 3]], [0, self.Tcurr[i][2, 3]], c='k')
            else:
                ax.plot([self.Tcurr[i - 1][0, 3], self.Tcurr[i][0, 3]],
                        [self.Tcurr[i - 1][1, 3], self.Tcurr[i][1, 3]],
                        [self.Tcurr[i - 1][2, 3], self.Tcurr[i][2, 3]], c='k')

        for b in range(len(self.Cpoints)):
            for i in range(1, 9):  # TODO might have to change the 9 to 5?
                for j in range(i, 9):
                    ax.plot([self.Cpoints[b][i][0], self.Cpoints[b][j][0]],
                            [self.Cpoints[b][i][1], self.Cpoints[b][j][1]],
                            [self.Cpoints[b][i][2], self.Cpoints[b][j][2]], c='b')

        for b in range(len(pointsObs)):
            for i in range(1, 9):
                for j in range(i, 9):
                    ax.plot([pointsObs[b][i][0], pointsObs[b][j][0]],
                            [pointsObs[b][i][1], pointsObs[b][j][1]],
                            [pointsObs[b][i][2], pointsObs[b][j][2]], c='r')

        # Format axes and display
        ax.set(xlim=(-0.6, .6), ylim=(-0.6, 0.6), zlim=(0, 1.2))
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')

        plt.show()
        return fig, ax

    def PlotSkeleton(self, ang):
        # Compute forward kinematics for ang
        self.ForwardKin(ang)

        # Create figure
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Draw links along coordinate frames
        for i in range(len(self.Tcurr)):
            ax.scatter(self.Tcurr[i][0, 3], self.Tcurr[i][1, 3], self.Tcurr[i][2, 3], c='k', marker='.')
            if i is 0:
                ax.plot([0, self.Tcurr[i][0, 3]], [0, self.Tcurr[i][1, 3]], [0, self.Tcurr[i][2, 3]], c='b')
            else:
                ax.plot([self.Tcurr[i - 1][0, 3], self.Tcurr[i][0, 3]], [self.Tcurr[i - 1][1, 3], self.Tcurr[i][1, 3]],
                        [self.Tcurr[i - 1][2, 3], self.Tcurr[i][2, 3]], c='k')

        plt.show()
        return fig, ax
