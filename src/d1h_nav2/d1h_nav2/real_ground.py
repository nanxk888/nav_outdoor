"""Bounded single-plane filter for initial qualified, locally planar real trials.

Not terrain traversability or a pit detector. A measured IMU-ground height is
mandatory; invalid planes produce NO free-space clearing output.
"""
import numpy as np


def split_ground(points, origin, height, height_tolerance=.15, reference_plane=None):
    p=np.asarray(points,dtype=float);o=np.asarray(origin,dtype=float)
    if not np.isfinite(height) or not .15<=height<=2.:
        raise ValueError('Set measured standing IMU height (0.15..2m) first')
    if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(o).all():raise ValueError('invalid points/origin')
    p=p[np.isfinite(p).all(axis=1)]
    distance=np.linalg.norm(p-o,axis=1)
    p=p[(distance>=.6)&(distance<=6.)]
    if len(p)<60:raise ValueError('too few scan points')
    expected=o[2]-height if reference_plane is None else float(np.dot(reference_plane[:2],o[:2])+reference_plane[2])
    if not .15<=o[2]-expected<=2.:raise ValueError('estimated IMU height outside qualified bounds')
    near=np.linalg.norm(p[:,:2]-o[:2],axis=1)<=4.
    prediction=expected if reference_plane is None else p[:,:2]@reference_plane[:2]+reference_plane[2]
    candidates=p[near & (np.abs(p[:,2]-prediction)<=height_tolerance)]
    if len(candidates)<40:raise ValueError('ground not observed in measured height band')
    rng=np.random.default_rng(17)
    if len(candidates)>2000:candidates=candidates[rng.choice(len(candidates),2000,replace=False)]
    best=None;score=0
    for _ in range(64):
        sample=candidates[rng.choice(len(candidates),3,replace=False)]
        matrix=np.c_[sample[:,:2]-o[:2],np.ones(3)]
        if abs(np.linalg.det(matrix))<.005:continue
        plane=np.linalg.solve(matrix,sample[:,2])
        if np.linalg.norm(plane[:2])>np.tan(np.deg2rad(20)) or abs(plane[2]-expected)>height_tolerance:continue
        error=np.abs(np.c_[candidates[:,:2]-o[:2],np.ones(len(candidates))]@plane-candidates[:,2])
        count=int((error<.015).sum())
        if count>score:best=plane;score=count
    if best is None or score<40 or score/len(candidates)<.45:raise ValueError('ground plane confidence insufficient')
    design=np.c_[candidates[:,:2]-o[:2],np.ones(len(candidates))]
    inliers=np.abs(design@best-candidates[:,2])<.015
    support=candidates[inliers]
    if np.ptp(support[:,0])<.5 or np.ptp(support[:,1])<.3:raise ValueError('ground support too narrow')
    plane=np.linalg.lstsq(design[inliers],support[:,2],rcond=None)[0]
    if np.linalg.norm(plane[:2])>np.tan(np.deg2rad(20)) or abs(plane[2]-expected)>height_tolerance:raise ValueError('refined ground invalid')
    clearance=p[:,2]-np.c_[p[:,:2]-o[:2],np.ones(len(p))]@plane
    # No rays from overhead ceilings or points substantially below the plane.
    clear=p[(clearance>=-.02)&(clearance<=.9)]
    nav=p[(clearance>=.03)&(clearance<=.9)&(np.linalg.norm(p-o,axis=1)<=5.)]
    plane[2]-=np.dot(plane[:2],o[:2]) # Return z=a*x+b*y+c in fixed LIO coordinates.
    return nav.astype('<f4'),clear.astype('<f4'),plane
