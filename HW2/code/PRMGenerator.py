import Locobot
import numpy as np
import matplotlib.pyplot as plt
import random
import pickle
import RobotUtil as rt
import time

# NOTE: Please set a random seed for your random joint generator so we can get the same path as you if we run your code!
random.seed(13)
# np.random.seed(0)

#Initialize robot object
mybot=Locobot.Locobot()

#Create environment obstacles 
pointsObs=[]
axesObs=[]

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.275,-0.15,0.]),[0.1,0.1,1.05])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.275,0.05,0.425]),[0.1,0.3,0.1])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.275,0.25,0.4]),[0.1,0.1,0.15])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.425,0.25,0.375]),[0.2,0.1,0.1])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[-0.1,0.0,0.675]),[0.45,0.15,0.1])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[-0.275,0.0,0.]),[0.1,1.0,1.25])
pointsObs.append(envpoints), axesObs.append(envaxes)

# base and camera mount for robot
envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.,0.0,0.05996]),[0.35004,0.3521,0.12276])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[-0.03768,0.0,0.36142]),[0.12001,0.26,0.5])
pointsObs.append(envpoints), axesObs.append(envaxes)


prmVertices=[]
#Create PRM - generate collision-free vertices
start = time.time()
num_samples = 2500 # you might need to change this value

for i in range(num_samples):
	# You will need to implement the sampler and collision detection
	q=mybot.SampleRobotConfig()
	if not mybot.DetectCollision(q, pointsObs, axesObs):
		prmVertices.append(q)

prmEdges=[]
#Create PRM - generate collision-free edges (and check for line-line collisions)


# Save the PRM
f = open("myPRM.p", 'wb')
pickle.dump(prmVertices, f)
pickle.dump(prmEdges, f)
pickle.dump(pointsObs, f)
pickle.dump(axesObs, f)
f.close

print("\n",len(prmVertices),": ", time.time()-start)





