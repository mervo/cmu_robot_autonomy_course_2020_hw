import os
import time
import pickle
import argparse
import numpy as np
import matplotlib.pyplot as plt
from os.path import dirname, join, abspath

# from pyrobot import Robot
import RobotUtil as rt
import Locobot


def FindNearest(prevPoints, newPoint):
    # You can use this function to find nearest neighbors in configuration space
    D = np.array([np.linalg.norm(np.array(point) - np.array(newPoint)) for point in prevPoints])
    return D.argmin()


def main(args):
    # NOTE: Please set a random seed for your random joint generator so we can get the same path as you if we run your code!
    np.random.seed(0)
    deg_to_rad = np.pi / 180.

    # Initialize robot object
    mybot = Locobot.Locobot()

    # Create environment obstacles
    pointsObs = []
    axesObs = []

    envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0, 0., 0.], [0.275, -0.15, 0.]), [0.1, 0.1, 1.05])
    pointsObs.append(envpoints), axesObs.append(envaxes)

    envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0, 0., 0.], [-0.1, 0.0, 0.675]), [0.45, 0.15, 0.1])
    pointsObs.append(envpoints), axesObs.append(envaxes)

    envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0, 0., 0.], [-0.275, 0.0, 0.]), [0.1, 1.0, 1.25])
    pointsObs.append(envpoints), axesObs.append(envaxes)

    # base and mount
    envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0, 0., 0.], [0., 0.0, 0.05996]), [0.35004, 0.3521, 0.12276])
    pointsObs.append(envpoints), axesObs.append(envaxes)

    envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0, 0., 0.], [-0.03768, 0.0, 0.36142]), [0.12001, 0.26, 0.5])
    pointsObs.append(envpoints), axesObs.append(envaxes)

    # Define initial pose
    qInit = [-80. * deg_to_rad, 0., 0., 0., 0.]

    # target box to grasp (You may only need the dimensions and pose, not the points and axes depending on your implementation)
    targetpoints, targetaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0, 0., 0.], [0.5, 0.0, 0.05]), [0.04, 0.04, 0.1])

    # Generate query for a block object (note random sampling in TGoal)
    QGoal = []
    num_grasp_points = 5  # You can adjust the number of grasp points you want to sample here
    while len(QGoal) < num_grasp_points:
        # TODO: Sample grasp points here, get their joint configurations, and check if they are valid
        pass

    # Create RRT graph to find path to a goal configuration
    rrtVertices = []
    rrtEdges = []

    # initialize with initial joint configuration and parent
    rrtVertices.append(qInit)
    rrtEdges.append(0)

    # Change these two hyperparameters as needed
    thresh = 1.5
    goal_thresh = 0.025
    goal_bias = 0.025
    num_samples = 3000

    FoundSolution = False

    for qGoal in QGoal:
        if FoundSolution:
            return

        while len(rrtVertices) < num_samples and not FoundSolution:
            print(len(rrtVertices))

            # NOTE: Remember to add a goal bias when you sample
            if np.random.uniform(0, 1) < goal_bias:  # probability of biasing our exploration to the goal
                # if direct path is blocked, too difficult to find a path
                qRand = qGoal
            else:
                # Use your sampler and collision detection from homework 2
                qRand = mybot.SampleRobotConfig()

            idNear = FindNearest(rrtVertices, qRand)  # find q_n
            qNear = rrtVertices[idNear]

            if np.linalg.norm(qRand - qNear) > thresh:  # limit step size for q_c
                qConnect = np.array(qNear) + (
                        thresh * (np.array(qRand) - np.array(qNear)) / np.linalg.norm(qRand - qNear))
            else:
                qConnect = qRand

            if not mybot.DetectCollision(qConnect, qNear, pointsObs, axesObs):  # Detect collisions
                rrtVertices.append(qConnect)  # Expand tree
                rrtEdges.append(idNear)  # id of parent node/vertex

            # Try to connect to goal
            idNear = FindNearest(rrtVertices, qGoal)  # find nearest point to goal
            if np.linalg.norm(qGoal - rrtVertices[idNear]) < goal_thresh and not mybot.DetectCollision(qGoal, qNear,
                                                                                                       pointsObs,
                                                                                                       axesObs):
                rrtVertices.append(qGoal)
                rrtEdges.append(idNear)
                break

    # TODO: Implement your RRT planner here

    if FoundSolution:

        # Extract path - TODO: add your path from your RRT after a solution has been found
        plan = []

        # Path shortening - TODO: implement path shortening in the for loop below
        num_iterations = 150  # change this hyperparameter as needed
        for i in range(num_iterations):
            # Sample vertices
            anchorA = np.random.randint(0, len(plan) - 2)
            anchorB = np.random.randint(anchorA + 1, len(plan) - 1)

            # Sample shift along edges
            shiftA = np.random.uniform(0, 1)
            shiftB = np.random.uniform(0, 1)

            # Compute test vertices
            candidateA = (1 - shiftA) * plan[anchorA] + shiftA * plan[anchorA + 1]
            candidateB = (1 - shiftB) * plan[anchorB] + shiftB * plan[anchorB + 1]

            # Shorten path if no collision
            if not mybot.DetectCollision(candidateA, candidateB, pointsObs, axesObs):
                while anchorB > anchorA:
                    plan.pop(anchorB)
                    anchorB -= 1
                plan.insert(anchorA + 1, candidateB)
                plan.insert(anchorA + 1, candidateA)

        if args.use_pyrobot:
            # Vizualize your plan in PyRobot
            common_config = {}
            common_config["scene_path"] = join(
                dirname(abspath(__file__)), "../scene/locobot_motion_hw3.ttt"
            )

            robot = Robot("vrep_locobot", common_config=common_config)

            # Execute plan
            for q in plan:
                robot.arm.set_joint_positions(q)

            # grasp block
            robot.gripper.close()

        else:
            # Visualize your Plan in matplotlib
            for q in plan:
                mybot.PlotCollisionBlockPoints(q, pointsObs)


    else:
        print("No solution found")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--use_pyrobot', type=bool, default=False)
    args = parser.parse_args()
    main(args)
