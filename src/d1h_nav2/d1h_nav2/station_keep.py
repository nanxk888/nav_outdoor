"""Bounded longitudinal/heading correction for a stationary wheeled robot."""
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class StationKeep:
    def __init__(self):
        self.reset()

    def reset(self):
        self.anchor = None
        self.integral = 0.0

    def command(self, x, y, yaw, vx, wz, dt):
        if self.anchor is None:
            self.anchor = (x, y, yaw)
        ax, ay, ayaw = self.anchor
        error = (ax-x)*math.cos(yaw)+(ay-y)*math.sin(yaw)
        self.integral = max(-0.3, min(0.3, self.integral+error*min(dt, 0.1)))
        v = 0.8*error+0.2*self.integral-0.5*vx
        w = 0.8*wrap(ayaw-yaw)-0.3*wz
        return max(-0.15, min(0.15, v)), max(-0.25, min(0.25, w))
