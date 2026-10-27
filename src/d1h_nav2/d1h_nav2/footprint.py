"""Exact circle/cell overlap, without an additional discretization margin."""
import math


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
