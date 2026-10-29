"""Bounded first-stage LIO tracking. Unknown cells are not traversable."""
import math
from d1h_nav2.footprint import circle_cells


def speed_limits(value):
    if not isinstance(value,dict) or set(value)!={'linear_mps','angular_rps'}:raise ValueError('Invalid local speed limits')
    for key,maximum in (('linear_mps',1.),('angular_rps',1.5)):
        v=value[key]
        if type(v) not in (int,float) or not math.isfinite(v) or not .1<=v<=maximum:raise ValueError('Local speed limits out of range')
    return dict(value)


def track(position,yaw,goal,points,grid,radius=.25,max_linear=.15,max_angular=.35):
    speed_limits(dict(linear_mps=max_linear,angular_rps=max_angular))
    if not points or len(points)>4000:raise ValueError('Missing/budget-exceeding full path')
    if not all(math.isfinite(v) for p in [position,goal,*points] for v in p[:2]) or not math.isfinite(yaw):raise ValueError('Invalid geometry')
    res,width,height,ox,oy,data=grid
    if not .1<=res<=1 or width*height!=len(data):raise ValueError('Invalid observed grid')
    def free(p):
        return all(0<=i<width and 0<=j<height and data[j*width+i]==0
                   for i,j in circle_cells(p,res,radius,(ox,oy)))
    if not free(position):raise ValueError('Current footprint not observed free')
    nearest=min(range(len(points)),key=lambda i:math.dist(position,points[i]))
    if math.dist(position,points[nearest])>.4:raise ValueError('Robot deviated from path')
    path=[position]+[tuple(p[:2]) for p in points[nearest:]]+[goal]
    if math.dist(path[-2],goal)>.4:raise ValueError('Path does not reach selected goal')
    # Validate the complete remaining swept circular footprint, not display samples.
    length=0.;carrot=goal
    for a,b in zip(path,path[1:]):
        d=math.dist(a,b)
        if length<.4<=length+d:
            t=(.4-length)/d;carrot=(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]))
        length+=d
        if length>6:raise ValueError('First-stage path exceeds 6m budget')
        for k in range(math.ceil(d/(res/2))+1):
            t=min(1,k*(res/2)/max(d,1e-9))
            if not free((a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]))):raise ValueError('Path footprint blocked or unknown')
    distance=math.dist(position,goal)
    if distance<=.15:return 0.,0.,True
    error=math.atan2(math.sin(math.atan2(carrot[1]-position[1],carrot[0]-position[0])-yaw),math.cos(math.atan2(carrot[1]-position[1],carrot[0]-position[0])-yaw))
    w=max(-max_angular,min(max_angular,1.2*error))
    # Distance-based braking envelope (0.5m/s² assumed), not a fixed 0.15m/s cap.
    v=0. if abs(error)>.45 else min(max_linear,math.sqrt(2*.5*max(0,distance-.15)))*max(0,math.cos(error))
    # Tracking a lookahead must not cut an unchecked shortcut across a corner.
    d=math.dist(position,carrot)
    for k in range(math.ceil(d/(res/2))+1):
        t=min(1,k*(res/2)/max(d,1e-9))
        if not free((position[0]+t*(carrot[0]-position[0]),position[1]+t*(carrot[1]-position[1]))):raise ValueError('Lookahead shortcut is blocked or unknown')
    # Check the commanded one-second motion arc, including circular turning footprint.
    for k in range(11):
        t=k*.1
        if abs(w)<1e-6:p=(position[0]+v*t*math.cos(yaw),position[1]+v*t*math.sin(yaw))
        else:p=(position[0]+v/w*(math.sin(yaw+w*t)-math.sin(yaw)),position[1]-v/w*(math.cos(yaw+w*t)-math.cos(yaw)))
        if not free(p):raise ValueError('Commanded motion footprint blocked or unknown')
    return v,w,False
