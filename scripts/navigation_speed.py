"""Shared final-output navigation limits; preserve curvature when limiting."""
import math

DEFAULT_LIMITS={'linear_mps':.15,'angular_rps':.35}


def validate_limits(value):
    if not isinstance(value,dict) or set(value)!=set(DEFAULT_LIMITS):raise ValueError('Expected linear_mps and angular_rps')
    for key,maximum in DEFAULT_LIMITS.items():
        v=value[key]
        if type(v) not in (int,float) or not math.isfinite(v) or not .01<=v<=maximum:
            raise ValueError(f'{key} must be 0.01..{maximum}')
    return dict(value)


def limit_velocity(v,w,limits):
    scale=min(1.,limits['linear_mps']/max(v,1e-12),limits['angular_rps']/max(abs(w),1e-12))
    return v*scale,w*scale
