"""Shortest-angle alignment phase; RPP remains the only velocity generator."""
import math

class HeadingAlign:
    def __init__(self):self.active=False;self.heading=None;self.error=0.
    def update(self,position,yaw,points):
        if not self.active:
            if not points:return False
            nearest=min(range(len(points)),key=lambda i:math.dist(position,points[i]))
            target=next((p for p in points[nearest:] if math.dist(position,p)>=.45),points[-1])
            if math.dist(position,target)<.05:return False
            self.heading=math.atan2(target[1]-position[1],target[0]-position[0])
        self.error=math.atan2(math.sin(self.heading-yaw),math.cos(self.heading-yaw))
        if not self.active and abs(self.error)>math.radians(120):self.active=True
        elif self.active and abs(self.error)<=math.radians(15):self.active=False
        return self.active
