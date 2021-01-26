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


def plotgraph(plan):
    joint1 = [vertex[0] for vertex in plan]
    joint2 = [vertex[1] for vertex in plan]
    joint3 = [vertex[2] for vertex in plan]
    joint4 = [vertex[3] for vertex in plan]
    joint5 = [vertex[4] for vertex in plan]
    plt.plot(joint1, label='Joint 1')
    plt.plot(joint2, label='Joint 2')
    plt.plot(joint3, label='Joint 3')
    plt.plot(joint4, label='Joint 4')
    plt.plot(joint5, label='Joint 5')
    plt.xlabel('Vertices')
    plt.ylabel('Joint position (rad)')
    plt.legend()
    plt.grid(True)


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
    QGoal = [[0.13876366374690352, 0.9348513069831005, 0.24135383319425296, -0.7107548655032749, -0.2074436520634615],
             [0.11637096014449981, 0.17635352859958434, -0.9676225093485422, 0.8954097389733264, -0.23031409129455668],
             [0.0898912092327068, 1.0368090085407666, -0.31998711981176925, -0.21691233556092593, -1.0711892601308433],
             [0.28547747340117624, 1.388111134554374, -1.2479019057961238, 0.15804122872763665, 0.36030220748791697],
             [0.7588114853264339, 1.0867893094233756, -0.9788200340674553, 0.4466013715102372, -0.324914738323657],
             [-0.19587524381782365, 0.05624873593393875, -0.7337248770587955, 0.8247973037441031, 0.6254096362496362],
             [1.0299997916116705, 0.8970534829277573, -0.29534302357944575, 0.047157919445781724, -0.3823099327048404],
             [0.9759771797292474, 0.8915728972864067, -0.25694804514852393, -0.6733819207835303, -0.8321323575070858],
             [-1.202314716538771, 0.4123345931861768, -0.019123982586583464, 1.0323444618282527, 1.3527878085212388],
             [0.01005852729145139, 1.2162720114076198, 0.15193571826372804, -1.5628775282435967, -1.1218560522309764]
             ]
    # num_grasp_points = 10  # You can adjust the number of grasp points you want to sample here
    # while len(QGoal) < num_grasp_points:
    #     # TODO: Sample grasp points here, get their joint configurations, and check if they are valid
    #     # done. to check
    #     tGoal = np.array([[1.0, 0.0, 0.0, 0.5],
    #                       [0.0, 1.0, 0.0, 0.0],
    #                       [0.0, 0.0, 1.0, 0.05 + np.random.uniform(-0.05, 0.05)],
    #                       [0., 0., 0., 1.]])
    #     q, err = mybot.IterInvKin(mybot.SampleRobotConfig(), tGoal)
    #     if np.linalg.norm(err[0:3]) < 0.01 and np.linalg.norm(err[4:6]) < 0.01 and mybot.RobotInLimits(q):
    #         if not mybot.DetectCollision(q, pointsObs, axesObs):
    #             QGoal.append(q)
    #             print("Found: " + str(len(QGoal)))
    #             print(q)

    QGoal = np.array(QGoal)

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
            break

        qGoal = np.array(qGoal)

        while len(rrtVertices) < num_samples and not FoundSolution:
            # NOTE: Remember to add a goal bias when you sample
            if np.random.uniform(0, 1) < goal_bias:  # probability of biasing our exploration to the goal
                # if direct path is blocked, too difficult to find a path
                qRand = qGoal
            else:
                # Use your sampler and collision detection from homework 2
                qRand = mybot.SampleRobotConfig()

            idNear = FindNearest(rrtVertices, qRand)  # find q_n
            qNear = rrtVertices[idNear]
            qNear = np.array(qNear)

            if np.linalg.norm(qRand - qNear) > thresh:  # limit step size for q_c
                qConnect = np.array(qNear) + (
                        thresh * (qRand - qNear) / np.linalg.norm(qRand - qNear))
            else:
                qConnect = qRand

            if not mybot.DetectCollisionEdge(qConnect, qNear, pointsObs, axesObs):  # Detect collisions
                rrtVertices.append(qConnect)  # Expand tree
                rrtEdges.append(idNear)  # id of parent node/vertex

            # Try to connect to goal
            idNear = FindNearest(rrtVertices, qGoal)  # find nearest point to goal
            if np.linalg.norm(qGoal - rrtVertices[idNear]) < goal_thresh and not mybot.DetectCollisionEdge(qGoal, qNear,
                                                                                                           pointsObs,
                                                                                                           axesObs):
                rrtVertices.append(qGoal)
                rrtEdges.append(idNear)
                FoundSolution = True
                break

    # TODO: Implement your RRT planner here

    if FoundSolution:
        # Extract path - TODO: add your path from your RRT after a solution has been found
        plan = []
        plan.insert(0, rrtVertices[-1])  # goal angles
        id = rrtEdges[-1]  # connected to goal
        while id:
            plan.insert(0, rrtVertices[id])
            id = rrtEdges[id]
        plan.insert(0, rrtVertices[0])  # initial angles
        for i, vertex in enumerate(plan):
            print(f'{i + 1} : {vertex}')

        plt.figure(figsize=(16, 9))
        plt.subplot(2, 1, 1)
        plotgraph(plan)
        plt.title('Before Path shortening')

        # Path shortening - TODO: implement path shortening in the for loop below
        num_iterations = 150  # change this hyperparameter as needed
        for i in range(num_iterations):
            # Sample vertices
            anchorA = np.random.randint(0, len(plan) - 2)  # choose a point but not the last 2 pts
            anchorB = np.random.randint(anchorA + 1, len(plan) - 1)  # choose a point after A but not the last pt

            # Sample shift along edges
            shiftA = np.random.uniform(0, 1)
            shiftB = np.random.uniform(0, 1)

            # Compute test vertices
            candidateA = (1 - shiftA) * np.array(plan[anchorA]) + shiftA * np.array(
                plan[anchorA + 1])  # in between A and A+1
            candidateB = (1 - shiftB) * np.array(plan[anchorB]) + shiftB * np.array(plan[anchorB + 1])

            # Shorten path if no collision
            if not mybot.DetectCollisionEdge(candidateA, candidateB, pointsObs, axesObs):
                while anchorB > anchorA:  # remove all the pts between A and B+1
                    plan.pop(anchorB)
                    anchorB -= 1
                plan.insert(anchorA + 1, candidateB)
                plan.insert(anchorA + 1, candidateA)
            print(f'Length of Plan: {len(plan)}')

        for i, vertex in enumerate(plan):
            print(f'{i + 1} : {vertex}')
        plt.subplot(2, 1, 2)
        plotgraph(plan)
        plt.title('After Path shortening')
        plt.savefig('paths.png')
        plt.show()

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
                # mybot.PlotCollisionBlockPoints(q, pointsObs)
                pass

    else:
        print("No solution found")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--use_pyrobot', type=bool, default=False)
    args = parser.parse_args()
    main(args)
