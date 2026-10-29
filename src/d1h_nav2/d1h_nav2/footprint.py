"""Exact circle/cell overlap, without an additional discretization margin."""
import math

BODY_POLYGON='[[0.13,0.23],[0.13,-0.23],[-0.25,-0.23],[-0.25,0.23]]'

def body_cells(position,resolution,heading,origin=(0.,0.)):
    """Positive-area rectangle/cell overlap; origin is the Livox IMU."""
    c,s=math.cos(heading),math.sin(heading)
    x,y=position[0]-.06*c,position[1]-.06*s
    rx=.19*abs(c)+.23*abs(s);ry=.19*abs(s)+.23*abs(c)
    ox,oy=origin;h=resolution/2
    for i in range(math.floor((x-rx-ox)/resolution),math.floor((x+rx-ox)/resolution)+1):
        for j in range(math.floor((y-ry-oy)/resolution),math.floor((y+ry-oy)/resolution)+1):
            dx=ox+(i+.5)*resolution-x;dy=oy+(j+.5)*resolution-y
            if (abs(dx)<rx+h-1e-12 and abs(dy)<ry+h-1e-12 and
                abs(dx*c+dy*s)<.19+h*(abs(c)+abs(s))-1e-12 and
                abs(-dx*s+dy*c)<.23+h*(abs(c)+abs(s))-1e-12):yield i,j

def body_path_free(points,resolution,free):
    for a,b in zip(points,points[1:]):
        yaw=math.atan2(b[1]-a[1],b[0]-a[0])
        steps=max(1,math.ceil(math.dist(a,b)/(resolution/8)))
        for k in range(steps+1):
            p=(a[0]+(b[0]-a[0])*k/steps,a[1]+(b[1]-a[1])*k/steps)
            if not all(free(cell) for cell in body_cells(p,resolution,yaw)):return False
    return bool(points)


def circle_cells(position,resolution,radius=.25,origin=(0.,0.)):
    x,y=position;ox,oy=origin
    for i in range(math.floor((x-radius-ox)/resolution),math.floor((x+radius-ox)/resolution)+1):
        for j in range(math.floor((y-radius-oy)/resolution),math.floor((y+radius-oy)/resolution)+1):
            dx=max(ox+i*resolution-x,0.,x-(ox+(i+1)*resolution))
            dy=max(oy+j*resolution-y,0.,y-(oy+(j+1)*resolution))
            # Boundary-only contact has zero area; no outside-cell padding.
            if dx*dx+dy*dy<radius*radius-1e-12:yield i,j


def path_free(points,resolution,radius,free):
    for a,b in zip(points,points[1:]):
        steps=max(1,math.ceil(math.dist(a,b)/(resolution/8)))
        for k in range(steps+1):
            p=(a[0]+(b[0]-a[0])*k/steps,a[1]+(b[1]-a[1])*k/steps)
            if not all(free(c) for c in circle_cells(p,resolution,radius)):return False
    return bool(points)
