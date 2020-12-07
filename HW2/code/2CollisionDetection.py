import RobotUtil


def origin_orientation_dims(xyz, rpy, dxdydz):
    return RobotUtil.BlockDesc2Points(RobotUtil.rpyxyz2H((rpy[0], rpy[1], rpy[2]), (xyz[0], xyz[1], xyz[2])),
                                      (dxdydz[0], dxdydz[1], dxdydz[2]))


ref_cuboid_corners, ref_cuboid_axes = origin_orientation_dims((0, 0, 0), (0, 0, 0), (3, 1, 2))
# print(ref_cuboid_corners)

test_origin = [(0, 1, 0), (1.5, -1.5, 0), (0, 0, -1), (3, 0, 0), (-1, 0, -2), (1.8, 0.5, 1.5), (0, -1.2, 0.4),
               (-0.8, 0, -0.5)]

test_orientation = [(0, 0, 0), (1, 0, 1.5), (0, 0, 0), (0, 0, 0), (0.5, 0, 0.4), (-0.2, 0.5, 0), (0, 0.785, 0.785),
                    (0, 0, 0.2)]

test_dims = [(0.8, 0.8, 0.8), (1, 3, 3), (2, 3, 1), (3, 1, 1), (2, 0.7, 2), (1, 3, 1), (1, 1, 1), (1, 0.5, 0.5)]

assert len(test_origin) == len(test_orientation) and len(test_orientation) == len(test_dims)

for i in range(len(test_origin)):
    corners_to_test, axes_to_test = origin_orientation_dims(test_origin[i], test_orientation[i], test_dims[i])
    print(RobotUtil.CheckBoxBoxCollision(ref_cuboid_corners, ref_cuboid_axes, corners_to_test, axes_to_test))

'''
False
True
True
True
True
False
True
True
'''