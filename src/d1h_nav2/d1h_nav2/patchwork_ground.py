"""Official Patchwork++ binding adapter for gravity-aligned native LIO XYZ.

The reference floor height is for initialization/height tracking, NOT a global
plane fit used to classify points. Unknown points never become clearing rays.
"""
import time
import math
import numpy as np


class PatchworkGround:
    def __init__(self,height,th_dist=.02,th_seeds=.04,min_range=.3,max_range=6.,min_ground=20,module=None):
        if not .15<=height<=2. or not .005<=th_dist<=.05 or not th_dist<=th_seeds<=.15:
            raise ValueError('Invalid Patchwork++ height/distance thresholds')
        if not 0<min_range<1 or not 3<=max_range<=10 or min_ground<6:raise ValueError('Invalid Patchwork++ range/support')
        if module is None:
            try:import pypatchworkpp as module
            except ImportError as exc:raise ValueError('Patchwork++ dependency missing: run bash scripts/install_patchworkpp.sh, then launch with scripts/launch_real_workspace.sh') from exc
        params=module.Parameters()
        params.sensor_height=height;params.verbose=False
        params.enable_RNR=False  # XYZ has no trustworthy intensity.
        params.enable_RVPF=True;params.enable_TGR=True
        params.min_range=min_range;params.max_range=max_range
        params.th_dist=th_dist;params.th_seeds=th_seeds
        params.th_dist_v=.05;params.th_seeds_v=.1
        params.num_min_pts=6;params.num_lpr=10;params.num_iter=3
        params.uprightness_thr=math.cos(math.radians(20))
        params.num_sectors_each_zone=[16,24,32,32];params.num_rings_each_zone=[2,2,2,2]
        self.engine=module.patchworkpp(params);self.initial_height=height;self.floor_z=None
        self.min_ground=min_ground;self.max_range=max_range;self.diagnostics={}
        self.configuration=dict(initial_imu_height_m=height,th_dist_m=th_dist,th_seeds_m=th_seeds,
            min_range_m=min_range,max_range_m=max_range,min_ground_points=min_ground,
            num_sectors_each_zone=[16,24,32,32],num_rings_each_zone=[2,2,2,2],RNR=False,RVPF=True,TGR=True,
            input_frame='gravity-aligned LIO; translated to IMU; virtual z height normalization')

    def split(self,points,origin):
        started=time.monotonic();p=np.asarray(points,dtype=np.float32);o=np.asarray(origin,dtype=float)
        if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(o).all():raise ValueError('Invalid Patchwork++ XYZ/origin')
        p=p[np.isfinite(p).all(axis=1)]
        distance=np.linalg.norm(p-o,axis=1);p=p[(distance>=.6)&(distance<=self.max_range)]
        if len(p)<60:raise ValueError('Patchwork++ insufficient input points')
        floor=o[2]-self.initial_height if self.floor_z is None else self.floor_z
        height=o[2]-floor
        if not .15<=height<=2.:raise ValueError('Patchwork++ estimated sensor height outside 0.15..2m')
        local=np.asarray(p-o,dtype=np.float32)
        # Virtual sensor z keeps the library's height prior constant while the
        # real IMU moves vertically. Returned indices refer to original p.
        local[:,2]+=height-self.initial_height
        cloud=np.zeros((len(p),4),dtype=np.float32);cloud[:,:3]=local
        algorithm_started=time.monotonic();self.engine.estimateGround(cloud)
        ground=np.asarray(self.engine.getGroundIndices(),dtype=np.int64).reshape(-1)
        nonground=np.asarray(self.engine.getNongroundIndices(),dtype=np.int64).reshape(-1)
        if any(np.any((ids<0)|(ids>=len(p))) for ids in (ground,nonground)):
            raise ValueError('Patchwork++ returned invalid indices')
        ground_mask=np.zeros(len(p),dtype=bool);ground_mask[ground]=True
        classified=ground_mask.copy();classified[nonground]=True
        self.diagnostics=dict(input_points=len(p),ground_points=len(ground),nonground_points=len(nonground),
            unclassified_points=int((~classified).sum()),algorithm_ms=(time.monotonic()-algorithm_started)*1000,
            estimated_imu_height_m=height,RNR_enabled=False)
        # Confirm a nearby floor-height anchor, without requiring a global
        # percentage of all clutter points to lie on one plane.
        support=p[ground_mask & (np.linalg.norm(p[:,:2]-o[:2],axis=1)<=2.5) & (np.abs(p[:,2]-floor)<=.15)]
        self.diagnostics['near_floor_support']=len(support)
        if len(support)<self.min_ground:raise ValueError('Patchwork++ nearby floor support insufficient: '+str(self.diagnostics))
        floor+=float(np.clip(np.median(support[:,2])-floor,-.03,.03))
        self.floor_z=floor;relative=p[:,2]-floor;radius=np.linalg.norm(p-o,axis=1)
        # Unclassified points may mark obstacles, but never clear free space.
        nav=p[(~ground_mask)&(relative>=.03)&(relative<=.9)&(radius<=5.)]
        clear=p[classified&(relative>=-.02)&(relative<=.9)]
        self.diagnostics['total_ms']=(time.monotonic()-started)*1000
        return nav.astype('<f4'),clear.astype('<f4'),np.array([0.,0.,floor])
