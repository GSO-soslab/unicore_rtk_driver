"""The few rotation helpers the driver needs, in plain numpy.

Drop-in replacements for the tf_transformations functions previously used
(same conventions: quaternions are [x, y, z, w], Euler angles are static-axis
'sxyz', i.e. R = Rz(yaw) @ Ry(pitch) @ Rx(roll)).  This removes the
tf_transformations / transforms3d pip dependency, which is not packaged for
ROS 2 Jazzy by default.
"""
import math

import numpy as np


def quaternion_from_euler(roll, pitch, yaw):
    """[x, y, z, w] for R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def quaternion_matrix(q):
    """4x4 homogeneous rotation matrix from [x, y, z, w]."""
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.identity(4)
    s = 2.0 / n
    m = np.identity(4)
    m[0, 0] = 1 - s * (y * y + z * z)
    m[0, 1] = s * (x * y - z * w)
    m[0, 2] = s * (x * z + y * w)
    m[1, 0] = s * (x * y + z * w)
    m[1, 1] = 1 - s * (x * x + z * z)
    m[1, 2] = s * (y * z - x * w)
    m[2, 0] = s * (x * z - y * w)
    m[2, 1] = s * (y * z + x * w)
    m[2, 2] = 1 - s * (x * x + y * y)
    return m


def euler_from_matrix(m):
    """(roll, pitch, yaw), 'sxyz', from a 3x3 or 4x4 rotation matrix."""
    m = np.asarray(m)[:3, :3]
    cy = math.hypot(m[0, 0], m[1, 0])
    if cy > 1e-9:
        roll = math.atan2(m[2, 1], m[2, 2])
        pitch = math.atan2(-m[2, 0], cy)
        yaw = math.atan2(m[1, 0], m[0, 0])
    else:  # gimbal lock
        roll = math.atan2(-m[1, 2], m[1, 1])
        pitch = math.atan2(-m[2, 0], cy)
        yaw = 0.0
    return roll, pitch, yaw


def quaternion_from_matrix(m):
    """[x, y, z, w] from a 3x3 or 4x4 rotation matrix."""
    m = np.asarray(m)[:3, :3]
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])
