import numpy as np
import math
import time
import random

import RobotUtil as rt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Locobot:

	def __init__(self):
		
		#Robot descriptor from URDF file (rpy xyz for each link)
		self.Rdesc=[
			[0, 0, 0, 0.0973, 0, 0.1730625], # From robot base to joint1
			[0, 0, 0, 0, 0, 0.04125],
			[0, 0, 0, 0.05, 0, 0.2],
			[0, 0, 0, 0.2002, 0, 0],
			[0, 0, 0, 0.063, 0.0001, 0],
			[0, 0, 0, 0.106525, 0, 0.0050143] # From joint5 to end-effector center
			]

		#Define the axis of rotation for each joint - use your self.axis from Homework 1
		self.axis=[]

		# joint limits for arm
		self.qmin=[-1.57, -1.57, -1.57, -1.57, -1.57] 
		self.qmax=[1.57, 1.57, 1.57, 1.57, 1.57] 

		#Robot collision blocks descriptor (base frame, (rpy xyz), length/width/height 
		# NOTE: Cidx and Cdesc are just the robot's link BB's 
		self.Cidx= [1,2,3,4] # which joint frame the BB should be defined in

		# xyz rpy poses of the robot arm blocks (use to create transforms)
		
		self.Cdesc=[[0.,0.,0., 0., 0., 0.09],
			[0.,0.,0., 0.075, 0.,0.],
			[0.,0.,0., 0.027, -0.012, 0.],
			[0.,0.,0., 0.055, 0.0, 0.01],			
	   	 	]
		
		# dimensions of robot arm blocks
		self.Cdim=[[0.05,0.05, 0.25],
			[0.25,0.05,0.05],
			[0.07,0.076,0.05],
			[0.11, 0.11, 0.07],			
		]

		#Set base coordinate frame as identity
		self.Tbase= [[1,0,0,0],
			[0,1,0,0],
			[0,0,1,0],
			[0,0,0,1]]
		
		#Initialize matrices
		self.Tlink=[] #Transforms for each link (const)
		self.Tjoint=[] #Transforms for each joint (init eye)
		self.Tcurr=[] #Coordinate frame of current (init eye)
		for i in range(len(self.Rdesc)):
			self.Tlink.append(rt.rpyxyz2H(self.Rdesc[i][0:3],self.Rdesc[i][3:6]))
			self.Tcurr.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])
			self.Tjoint.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])

		self.Tlink[0]=np.matmul(self.Tbase,self.Tlink[0])

		self.J=np.zeros((6,5))
		
		self.q=[0.,0.,0.,0.,0.,0.]
		self.ForwardKin([0.,0.,0.,0.,0.])

		self.Tblock=[] #Transforms for each arm block
		self.Tcoll=[]  #Coordinate frame of current collision block

		self.Cpoints=[]
		self.Caxes=[]

		for i in range(len(self.Cdesc)):
			self.Tblock.append(rt.rpyxyz2H(self.Cdesc[i][0:3],self.Cdesc[i][3:6]))
			self.Tcoll.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])
			
			self.Cpoints.append(np.zeros((3,4)))
			self.Caxes.append(np.zeros((3,3)))


	def ForwardKin(self,ang):
		# Put you FK code from Homework 1 here
		self.q[0:-1]=ang		
		
		return self.Tcurr, self.J


	def IterInvKin(self,ang,TGoal):
		# Put you IK code from Homework 1 here

		self.ForwardKin(ang)		
		
		return self.q[0:-1], Err


	def SampleRobotConfig(self):
		# implement random sampling of robot joint configurations
		q=[]		
		
		return q


	def CompCollisionBlockPoints(self,ang):
		# Use your FK implementation here to compute collision boxes for the robot arm 
		
		pass

		
	def DetectCollision(self, ang, pointsObs, axesObs):	
		# implement collision detection using CompCollisionBlockPoints() and rt.CheckBoxBoxCollision()

		return False


	def DetectCollisionEdge(self, ang1, ang2, pointsObs, axesObs):
		# Detects if an edge is valid or in collision
		for s in np.linspace(0,1,50):
			ang= [ang1[k]+s*(ang2[k]-ang1[k]) for k in range(len(ang1))] 	

			self.CompCollisionBlockPoints(ang)

			for i in range(len(self.Cpoints)):
				for j in range(len(pointsObs)):
					if rt.CheckBoxBoxCollision(self.Cpoints[i],self.Caxes[i],pointsObs[j], axesObs[j]):
						return True

		return False

	def PlotCollisionBlockPoints(self,ang,pointsObs=[]):
		# This is a plotting function to visualize you robot with obstacles

		#Compute collision block points
		self.CompCollisionBlockPoints(ang)
		#Create figure
		fig =plt.figure()
		ax = fig.add_subplot(111,projection='3d')


		#Draw links along coordinate frames 
		for i in range(len(self.Tcurr)):
			ax.scatter(self.Tcurr[i][0,3], self.Tcurr[i][1,3], self.Tcurr[i][2,3], c='k', marker='.')
			if i is 0:
				ax.plot([0,self.Tcurr[i][0,3]], [0,self.Tcurr[i][1,3]], [0,self.Tcurr[i][2,3]], c='k')
			else:
				ax.plot([self.Tcurr[i-1][0,3],self.Tcurr[i][0,3]], 
					[self.Tcurr[i-1][1,3],self.Tcurr[i][1,3]], 
					[self.Tcurr[i-1][2,3],self.Tcurr[i][2,3]], c='k')

		for b in range(len(self.Cpoints)):
			for i in range(1,9): #TODO might have to change the 9 to 5?
				for j in range(i,9):
					ax.plot([self.Cpoints[b][i][0],  self.Cpoints[b][j][0]], 
						[self.Cpoints[b][i][1],  self.Cpoints[b][j][1]], 
						[self.Cpoints[b][i][2],  self.Cpoints[b][j][2]], c='b')
				
		for b in range(len(pointsObs)):
			for i in range(1,9):
				for j in range(i,9):
					ax.plot([pointsObs[b][i][0],  pointsObs[b][j][0]], 
						[pointsObs[b][i][1],  pointsObs[b][j][1]], 
						[pointsObs[b][i][2],  pointsObs[b][j][2]], c='r')


		#Format axes and display
		ax.set(xlim=(-0.6, .6), ylim=(-0.6, 0.6), zlim=(0,1.2))
		ax.set_xlabel('X-axis')
		ax.set_ylabel('Y-axis')
		ax.set_zlabel('Z-axis')

		plt.show()
		return fig, ax


	def PlotSkeleton(self,ang):
		#Compute forward kinematics for ang
		self.ForwardKin(ang)

		#Create figure
		fig =plt.figure()
		ax = fig.add_subplot(111,projection='3d')

		#Draw links along coordinate frames 
		for i in range(len(self.Tcurr)):
			ax.scatter(self.Tcurr[i][0,3], self.Tcurr[i][1,3], self.Tcurr[i][2,3], c='k', marker='.')
			if i is 0:
				ax.plot([0,self.Tcurr[i][0,3]], [0,self.Tcurr[i][1,3]], [0,self.Tcurr[i][2,3]], c='b')
			else:
				ax.plot([self.Tcurr[i-1][0,3],self.Tcurr[i][0,3]], [self.Tcurr[i-1][1,3],self.Tcurr[i][1,3]], [self.Tcurr[i-1][2,3],self.Tcurr[i][2,3]], c='k')

		plt.show()
		return fig, ax


