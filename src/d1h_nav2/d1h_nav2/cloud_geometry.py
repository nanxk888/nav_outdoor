"""Rigid body projection retaining physical roll, pitch and sensor height."""
import numpy as np


def transform_xyz(points, translation, quaternion):
    x, y, z, w = quaternion
    norm = x*x+y*y+z*z+w*w
    if norm < 1e-12:
        raise ValueError('Invalid transform quaternion')
    s = 2.0/norm
    rotation = np.array([
        [1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
        [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)]])
    return points @ rotation.T + np.asarray(translation)
