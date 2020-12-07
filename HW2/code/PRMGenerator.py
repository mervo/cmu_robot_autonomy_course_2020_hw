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

# Initialize robot object
mybot = Locobot.Locobot()

# Create environment obstacles
pointsObs = []
axesObs = []

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0, 0., 0.], [0.275, -0.15, 0.]), [0.1, 0.1, 1.05])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0, 0., 0.], [0.275, 0.05, 0.425]), [0.1, 0.3, 0.1])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0, 0., 0.], [0.275, 0.25, 0.4]), [0.1, 0.1, 0.15])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0, 0., 0.], [0.425, 0.25, 0.375]), [0.2, 0.1, 0.1])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0, 0., 0.], [-0.1, 0.0, 0.675]), [0.45, 0.15, 0.1])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0, 0., 0.], [-0.275, 0.0, 0.]), [0.1, 1.0, 1.25])
pointsObs.append(envpoints), axesObs.append(envaxes)

# base and camera mount for robot
envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0, 0., 0.], [0., 0.0, 0.05996]), [0.35004, 0.3521, 0.12276])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0, 0., 0.], [-0.03768, 0.0, 0.36142]), [0.12001, 0.26, 0.5])
pointsObs.append(envpoints), axesObs.append(envaxes)

# Create PRM - generate collision-free edges (and check for line-line collisions)
# Temporarily connect q_i and q_j to the graph and search for a path within the graph
prmVertices = []
prmEdges = []

start = time.time()
num_samples = 300  # you might need to change this value
radius_threshold = 2.0

for i in range(num_samples):
    print(i)
    # You will need to implement the sampler and collision detection
    q = mybot.SampleRobotConfig()  # sample vertex
    if not mybot.DetectCollision(q, pointsObs, axesObs):
        prmVertices.append(q)  # add collision-free vertex

        prmEdges.append([])
        for j in range(len(prmVertices) - 1):  # find (radius) neighbours
            if np.linalg.norm(np.array(prmVertices[-1]) - np.array(prmVertices[j])) < radius_threshold:
                if not mybot.DetectCollisionEdge(prmVertices[-1], prmVertices[j], pointsObs,
                                                 axesObs):  # detect collisions along edge between prev and cur point

                    # add collision-free edge
                    prmEdges[-1].append(j)
                    prmEdges[j].append(len(prmVertices) - 1)

# Save the PRM
f = open("myPRM.p", 'wb')
pickle.dump(prmVertices, f)
pickle.dump(prmEdges, f)
pickle.dump(pointsObs, f)
pickle.dump(axesObs, f)
f.close

print("\n", len(prmVertices), ": ", time.time() - start)
