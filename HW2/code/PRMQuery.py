from __future__ import print_function
import argparse
import os
import sys
from os.path import dirname, join, abspath

import numpy as np
import time
import pickle
import matplotlib.pyplot as plt
import random

# from pyrobot import Robot
import Locobot
import RobotUtil as rt

# NOTE: Please set a random seed for your random joint generator so we can get the same path as you if we run your code!
random.seed(13)
# np.random.seed(0)

from scipy.interpolate import interp1d


def main(args):
    # Initialize robot object
    mybot = Locobot.Locobot()

    # open road map
    f = open("myPRM.p", 'rb')
    prmVertices = pickle.load(f)
    prmEdges = pickle.load(f)
    pointsObs = pickle.load(f)
    axesObs = pickle.load(f)
    f.close

    # define start and goal
    deg_to_rad = np.pi / 180.
    qInit = [-80. * deg_to_rad, 0., 0., 0., 0.]
    qGoal = [0., 60 * deg_to_rad, -75 * deg_to_rad, -75 * deg_to_rad, 0.]

    plan = []
    """ Write the code for querying the PRM here and return a path from start to goal """
    neighInit = []
    neighGoal = []
    heuristic = []
    parent = []
    for i in range(len(prmVertices)):
        if np.linalg.norm(np.array(prmVertices[i]) - np.array(qInit)) < 2.:
            if not mybot.DetectCollisionEdge(prmVertices[i], qInit, pointsObs, axesObs):
                neighInit.append(i)
        if np.linalg.norm(np.array(prmVertices[i]) - np.array(qGoal)) < 2.:
            if not mybot.DetectCollisionEdge(prmVertices[i], qGoal, pointsObs, axesObs):
                neighGoal.append(i)
        heuristic.append(np.linalg.norm(np.array(prmVertices[i]) - np.array(qGoal)))
        parent.append([])
    activenodes = neighInit
    bestscore = 0
    while bestscore < 1000 and not any([g in activenodes for g in neighGoal]):
        bestscore = 1000
        for i in range(len(activenodes)):
            for j in range(len(prmEdges[activenodes[i]])):
                if prmEdges[activenodes[i]][j] not in activenodes:
                    if heuristic[prmEdges[activenodes[i]][j]] < bestscore:
                        bestscore = heuristic[prmEdges[activenodes[i]][j]]
                        bestcandi = prmEdges[activenodes[i]][j]
                        bestparent = activenodes[i]
        if bestscore < 1000:
            activenodes.append(bestcandi)
            parent[bestcandi] = bestparent
    plan = [activenodes[-1]]
    prevstep = parent[plan[0]]
    while prevstep:
        plan.insert(0, prevstep)
        prevstep = parent[plan[0]]
    print(f'Plan: {plan}')
    MyPlan = []
    MyPlan.append(qInit)
    for i in range(len(plan)):
        MyPlan.append(prmVertices[plan[i]])
    MyPlan.append(qGoal)

    print(f'MyPlan: {MyPlan}')

    # Interpolate for smooth trajectory
    num_steps_between = 10
    MyPlanInterpolated = []
    for i in range(len(MyPlan) - 1):
        linfit = interp1d([0, num_steps_between - 1], np.vstack([MyPlan[i], MyPlan[i + 1]]), axis=0)
        for j in range(num_steps_between):
            MyPlanInterpolated.append(list(linfit(j)))

    print(f'MyPlanInterpolated: {MyPlanInterpolated}')
    MyPlan = MyPlanInterpolated

    if args.use_pyrobot:
        # Vizualize your plan in PyRobot
        common_config = {}
        common_config["scene_path"] = join(
            dirname(abspath(__file__)), "../scene/locobot_motion.ttt"
        )

        robot = Robot("vrep_locobot", common_config=common_config)

        for q in MyPlan:
            robot.arm.set_joint_positions(q)

    else:
        # Visualize your Plan in matplotlib
        pass
        # for q in MyPlan:
        #     mybot.PlotCollisionBlockPoints(q, pointsObs)

    # Plot time history plot for joints 1 through 5 that show the path produced by your PRM
    # The time history plot is a bit difficult to interpret in 2D, but it does give an idea of how much each joint position is changing between each point on your path. Ideally you don't want large changes/jumps in position from point to point
    import seaborn as sns
    joints = list(zip(*MyPlan))
    x = np.linspace(1, len(joints[0]), len(joints[0]))
    sns.set()
    plt.xlabel("Time Step")
    plt.ylabel("Joint Configuration (rad)")
    plt.title("Time History Plot of Joints")
    joint = 0
    for y in joints:
        plt.plot(x, y, label=f'Joint {joint}')
        joint += 1
    plt.legend()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--use_pyrobot', type=bool, default=False)
    args = parser.parse_args()
    main(args)
