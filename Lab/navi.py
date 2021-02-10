from pyrobot import Robot

base_config_dict={'base_controller': 'proportional'}
robot = Robot('locobot', base_config=base_config_dict)

target_position = [1.0, 1.0, 0.0] 
robot.base.go_to_relative(target_position, smooth=False, close_loop=True)
