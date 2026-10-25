"""Planar reference from tilted IMU +X (operator-confirmed robot forward)."""
import math


def heading(q):
    if len(q)!=4 or not all(math.isfinite(v) for v in q):raise ValueError('Invalid quaternion')
    norm=math.sqrt(sum(v*v for v in q))
    if abs(norm-1)>.01:raise ValueError('Quaternion is not normalized')
    x,y,z,w=(v/norm for v in q)
    hx,hy=1-2*(y*y+z*z),2*(x*y+w*z)
    if math.hypot(hx,hy)<.3:raise ValueError('Forward axis too close to vertical')
    return math.atan2(hy,hx)


def registration(gps_xy,lio_xy,imu_heading,bearing_deg):
    if not all(math.isfinite(v) for v in (*gps_xy,*lio_xy,imu_heading,bearing_deg)) or not 0<=bearing_deg<360:
        raise ValueError('Invalid registration numbers')
    yaw=math.pi/2-math.radians(bearing_deg)-imu_heading
    c,s=math.cos(yaw),math.sin(yaw)
    return (gps_xy[0]-c*lio_xy[0]+s*lio_xy[1],gps_xy[1]-s*lio_xy[0]-c*lio_xy[1],yaw)
