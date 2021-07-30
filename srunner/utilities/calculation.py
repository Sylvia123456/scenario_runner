import math
import numpy as np


def get_rotate_angle(v):
    normal = np.array([1, 0])
    ln = np.sqrt(normal.dot(normal))
    lv = np.sqrt(v.dot(v))
    cos_angle = v.dot(normal) / (ln * lv)
    angle_value = np.arccos(cos_angle)
    angle_value = angle_value * 360 / (2 * np.pi)
    return angle_value


def get_clock_angle(v):
    normal = np.array([1, 0])
    the_normal = np.linalg.norm(v) * np.linalg.norm(normal)
    # 叉乘
    rho = np.rad2deg(np.arcsin(np.cross(v, normal) / the_normal))
    # 点乘
    theta = np.rad2deg(np.arccos(np.dot(v, normal) / the_normal))
    if rho < 0:
        return - theta
    else:
        return theta


def angle(v1, v2):
    angle1 = math.atan2(v1.y, v1.x)
    angle1 = int(angle1 * 180 / math.pi)
    # print(angle1)
    angle2 = math.atan2(v2.y, v2.x)
    angle2 = int(angle2 * 180 / math.pi)
    # print(angle2)
    if angle1 * angle2 >= 0:
        included_angle = abs(angle1 - angle2)
    else:
        included_angle = abs(angle1) + abs(angle2)
        if included_angle > 180:
            included_angle = 360 - included_angle
    return included_angle
