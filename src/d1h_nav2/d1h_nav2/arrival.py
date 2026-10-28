"""Planar arrival latch: zero first, then confirm fresh LIO standstill."""
import math

class Arrival:
    def __init__(self):self.since=None;self.previous=None;self.stable=None
    def update(self,position,yaw,stamp,distance,now):
        if self.since is None:
            if distance>.15:return 'TRACKING'
            self.since=now
        if distance>.25:return 'OVERSHOOT'
        if self.previous and stamp>self.previous[0]:
            t,p,a=self.previous;dt=stamp-t
            slow=dt<=.4 and math.dist(position,p)/dt<=.03 and abs(math.atan2(math.sin(yaw-a),math.cos(yaw-a)))/dt<=.05
            if slow:
                if self.stable is None:self.stable=stamp
                if stamp-self.stable>=.7 and distance<=.15:return 'SETTLED'
            else:self.stable=None
        if self.previous is None or stamp>self.previous[0]:self.previous=(stamp,position,yaw)
        return 'TIMEOUT' if now-self.since>5 else 'BRAKING'

def approach_cap(distance):
    braking=max(.03,math.sqrt(max(0.,2*.3*(distance-.15))))
    return min(.15,braking) if distance<.8 else braking
