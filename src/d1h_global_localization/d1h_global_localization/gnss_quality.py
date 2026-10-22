"""Pure GNSS acceptance rules; covariance is ENU m^2, never HDOP meters."""
import math
import numpy as np


class GnssGate:
    def __init__(self, max_age=1.5, future_tolerance=.1, min_std=1., max_std=10.,
                 allow_unknown=False, fallback_std=5., max_speed=2., jump_margin=10.):
        if not (0<min_std<=fallback_std<=max_std and max_age>0 and max_speed>0 and jump_margin>0):
            raise ValueError('Invalid GNSS acceptance parameters')
        self.max_age=max_age;self.future_tolerance=future_tolerance
        self.min_std=min_std;self.max_std=max_std;self.allow_unknown=allow_unknown
        self.fallback_std=fallback_std;self.max_speed=max_speed;self.jump_margin=jump_margin
        self.last=None

    def check(self, *, now, stamp, frame, status, lat, lon, covariance, covariance_type):
        def reject(reason):return False,reason,None,False
        if frame!='gps_antenna':return reject('unexpected_frame')
        if status not in (0,1,2):return reject('no_fix')
        if not all(math.isfinite(x) for x in (lat,lon,stamp,now)) or not (-90<lat<90 and -180<=lon<=180):
            return reject('invalid_coordinates_or_time')
        if stamp<=0 or now-stamp>self.max_age or now-stamp < -self.future_tolerance:
            return reject('stale_or_future_fix')
        if self.last and stamp<=self.last[0]:return reject('non_increasing_stamp')
        fallback=False
        if covariance_type==0:
            if not self.allow_unknown:return reject('unknown_covariance')
            cov=np.eye(3)*self.fallback_std**2;fallback=True
        elif covariance_type in (1,2,3):
            cov=np.asarray(covariance,dtype=float).reshape(3,3)
            if not np.isfinite(cov).all() or not np.allclose(cov,cov.T,atol=1e-6):
                return reject('invalid_covariance')
            if np.linalg.eigvalsh(cov).min() < -1e-8 or np.any(np.diag(cov)<=0):
                return reject('nonpositive_covariance')
            if math.sqrt(np.linalg.eigvalsh(cov[:2,:2]).max())>self.max_std:
                return reject('horizontal_uncertainty_too_large')
            # Conservative inflation only; do not invent RTK confidence.
            cov=cov+np.diag(np.maximum(0.,self.min_std**2-np.diag(cov)))
        else:return reject('invalid_covariance_type')
        if self.last:
            t,old_lat,old_lon=self.last
            dy=6378137.*math.radians(lat-old_lat)
            dl=(lon-old_lon+180)%360-180
            dx=6378137.*math.radians(dl)*math.cos(math.radians((lat+old_lat)/2))
            if math.hypot(dx,dy)>self.jump_margin+self.max_speed*(stamp-t):
                return reject('position_jump')
        self.last=(stamp,lat,lon)
        return True,'accepted_fallback_covariance' if fallback else 'accepted',cov.ravel().tolist(),fallback


def antenna_enu(lat,lon,lat0,lon0):
    return (6378137.*math.radians(lon-lon0)*math.cos(math.radians(lat0)),
            6378137.*math.radians(lat-lat0))
