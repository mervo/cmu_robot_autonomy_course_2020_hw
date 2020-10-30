# Import system libraries
from __future__ import print_function
import argparse
import os
import sys
from os.path import dirname, join, abspath

# Import application libraries
import numpy as np
from pyrobot import Robot
from pyrep.const import ObjectType, JointMode


def main(args):
    common_config = {}
    common_config["scene_path"] = join(
        dirname(abspath(__file__)), "hw1_locobot_fk_ik.ttt"
    )
   
    deg_to_rad = np.pi/180.

    robot = Robot("vrep_locobot", common_config=common_config)

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

    robot.arm.set_joint_positions(joint_targets[2])
    pos, rot, quat = robot.arm.get_ee_pose()
    print('ee position', pos)
    print('ee rot', rot)

    for i in range(100): #NOTE: can change this value (100) to run the sim for longer or short amount of time
        # Note: this commands updates the UI as the simulation runs
        robot.arm.sim.step_ui() #see https://github.com/kalyanvasudev/PyRep/blob/master/pyrep/pyrep.py
 
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    args = parser.parse_args()
    main(args)