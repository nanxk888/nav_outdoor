"""Ground-status timing contract shared by publisher and local executor."""
import math

STATUS_PERIOD_SEC = .1
MAX_AGE_SEC = .5


def require_ground(status,receive_age):
    if not math.isfinite(receive_age) or not 0<=receive_age<MAX_AGE_SEC:
        raise ValueError(f'Ground status stale: receive_age={receive_age:.3f}s limit={MAX_AGE_SEC:.3f}s')
    if status.get('ready') is not True:
        raise ValueError('Ground filtering rejected: '+str(status.get('reason','no valid ground status')))
    value=status.get('measurement_age_sec')
    if type(value) not in (int,float) or not math.isfinite(value):
        raise ValueError('Ground measurement age missing/invalid')
    age=value+receive_age
    if value<-.05 or not -.05<=age<MAX_AGE_SEC:
        raise ValueError(f'Ground measurement stale/invalid: age={age:.3f}s reported_age={value:.3f}s receive_age={receive_age:.3f}s limit={MAX_AGE_SEC:.3f}s')
