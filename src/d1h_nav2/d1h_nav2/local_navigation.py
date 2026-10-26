"""Bounded first-stage LIO tracking. Unknown cells are not traversable."""
import math


def track(position,yaw,goal,points,grid,radius=.6):
    if not points or len(points)>4000:raise ValueError('Missing/budget-exceeding full path')
    if not all(math.isfinite(v) for p in [position,goal,*points] for v in p[:2]) or not math.isfinite(yaw):raise ValueError('Invalid geometry')
    res,width,height,ox,oy,data=grid
    if not .1<=res<=1 or width*height!=len(data):raise ValueError('Invalid observed grid')
    n=math.ceil((radius+res*.71)/res)
    stencil=[(i,j) for i in range(-n,n+1) for j in range(-n,n+1) if math.hypot(i,j)*res<=radius+res*.71]
    def free(p):
        cx,cy=math.floor((p[0]-ox)/res),math.floor((p[1]-oy)/res)
        return all(0<=cx+i<width and 0<=cy+j<height and data[(cy+j)*width+cx+i]==0 for i,j in stencil)
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
    w=max(-.35,min(.35,1.2*error))
    v=0. if abs(error)>.45 else min(.15,.4*max(0,distance-.1))*max(0,math.cos(error))
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
