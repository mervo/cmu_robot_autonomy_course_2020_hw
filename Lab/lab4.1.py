from pyrobot import Robot
import time

robot = Robot('locobot', base_config={'base_controller': 'ilqr'})
current_state = robot.base.get_state('odom')
print(current_state)

#robot.base.go_to_absolute([0.3, 0.0, 0.1])

current_state = robot.base.get_state('odom')
print(current_state)
time.sleep(5)

robot.base.go_to_relative([0.1, 0.0, 0.0])

current_state = robot.base.get_state('odom')
print(current_state)
time.sleep(5)

#robot.base.go_to_absolute([0.0, 0.0, 0.0])

current_state = robot.base.get_state('odom')
print(current_state)