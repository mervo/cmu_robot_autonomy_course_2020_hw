import numpy as np
import math


def rpyxyz2H(rpy, xyz):
    Ht = [[1, 0, 0, xyz[0]],
          [0, 1, 0, xyz[1]],
          [0, 0, 1, xyz[2]],
          [0, 0, 0, 1]]

    Hx = [[1, 0, 0, 0],
          [0, math.cos(rpy[0]), -math.sin(rpy[0]), 0],
          [0, math.sin(rpy[0]), math.cos(rpy[0]), 0],
          [0, 0, 0, 1]]

    Hy = [[math.cos(rpy[1]), 0, math.sin(rpy[1]), 0],
          [0, 1, 0, 0],
          [-math.sin(rpy[1]), 0, math.cos(rpy[1]), 0],
          [0, 0, 0, 1]]

    Hz = [[math.cos(rpy[2]), -math.sin(rpy[2]), 0, 0],
          [math.sin(rpy[2]), math.cos(rpy[2]), 0, 0],
          [0, 0, 1, 0],
          [0, 0, 0, 1]]

    H = np.matmul(np.matmul(np.matmul(Ht, Hx), Hy), Hz)

    return H


def R2axisang(R):
    ang = math.acos((R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2)
    Z = np.linalg.norm([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    x = (R[2, 1] - R[1, 2]) / Z
    y = (R[0, 2] - R[2, 0]) / Z
    z = (R[1, 0] - R[0, 1]) / Z

    return [x, y, z], ang


def BlockDesc2Points(H, Dim):
    center = H[0:3, 3]
    axes = [H[0:3, 0], H[0:3, 1], H[0:3, 2]]

    corners = [
        center,
        center + (axes[0] * Dim[0] / 2.) + (axes[1] * Dim[1] / 2.) + (axes[2] * Dim[2] / 2.),
        center + (axes[0] * Dim[0] / 2.) + (axes[1] * Dim[1] / 2.) - (axes[2] * Dim[2] / 2.),
        center + (axes[0] * Dim[0] / 2.) - (axes[1] * Dim[1] / 2.) + (axes[2] * Dim[2] / 2.),
        center + (axes[0] * Dim[0] / 2.) - (axes[1] * Dim[1] / 2.) - (axes[2] * Dim[2] / 2.),
        center - (axes[0] * Dim[0] / 2.) + (axes[1] * Dim[1] / 2.) + (axes[2] * Dim[2] / 2.),
        center - (axes[0] * Dim[0] / 2.) + (axes[1] * Dim[1] / 2.) - (axes[2] * Dim[2] / 2.),
        center - (axes[0] * Dim[0] / 2.) - (axes[1] * Dim[1] / 2.) + (axes[2] * Dim[2] / 2.),
        center - (axes[0] * Dim[0] / 2.) - (axes[1] * Dim[1] / 2.) - (axes[2] * Dim[2] / 2.)
    ]
    # returns corners of BB and axes
    return corners, axes


def CheckPointOverlap(pointsA, pointsB, axis):
    # check if points are overlapping

    # project points onto axis
    projPointsA = np.matmul(axis, np.transpose(pointsA))
    projPointsB = np.matmul(axis, np.transpose(pointsB))

    # check overlap
    maxA = np.max(projPointsA)
    minA = np.min(projPointsA)
    maxB = np.max(projPointsB)
    minB = np.min(projPointsB)

    if maxA <= maxB and maxA >= minB:
        return True

    if minA <= maxB and minA >= minB:
        return True

    if maxA >= maxB >= minA:
        return True

    if minB <= maxA and minB >= minA:
        return True

    return False


def CheckBoxBoxCollision(pointsA, axesA, pointsB, axesB):
    # check collision between two boxes

    # sphere check - first point is the box's center
    # coarse check first - if the distance between the centers is > than the sum of the 2 radii; cannot be colliding
    if np.linalg.norm(pointsA[0] - pointsB[0]) > (
            np.linalg.norm(pointsA[0] - pointsA[1]) + np.linalg.norm(pointsB[0] - pointsB[1])):
        return False

    # surface normal check
    for i in range(3):
        if not CheckPointOverlap(pointsA, pointsB, axesA[i]):  # check all axes of object A
            return False

    for j in range(3):
        if not CheckPointOverlap(pointsA, pointsB, axesB[j]):
            return False

    # edge to edge check
    for i in range(3):
        for j in range(3):
            if not CheckPointOverlap(pointsA, pointsB, np.cross(axesA[i], axesB[j])):
                return False

    return True
