"""Trimmed multi-scale 6-DoF point-to-point ICP (prototype, no motion prior)."""
import numpy as np
from scipy.spatial import cKDTree


def downsample(points, resolution, limit=None):
    points = np.asarray(points, dtype=float)
    points = points[np.isfinite(points).all(axis=1)]
    _, ids = np.unique(np.floor(points/resolution).astype(np.int64), axis=0, return_index=True)
    points = points[np.sort(ids)]
    if limit and len(points)>limit:
        points = points[np.linspace(0,len(points)-1,limit,dtype=int)]
    return points


def register(points, target, initial, stages=(1.0,.5,.25), iterations=18):
    transform = initial.copy()
    source = downsample(points,.10,2500)
    tree = target if isinstance(target,cKDTree) else cKDTree(target)
    if len(source)<100:
        return transform, dict(accepted=False,rmse=1e6,overlap=0.,points=len(source))
    for radius in stages:
        for _ in range(iterations):
            moved = source@transform[:3,:3].T+transform[:3,3]
            distances, ids = tree.query(moved, distance_upper_bound=radius,workers=1)
            valid = distances<radius
            if valid.sum()<80:
                break
            trim = np.quantile(distances[valid],.85)
            valid &= distances<=max(trim,.03)
            a,b = moved[valid],tree.data[ids[valid]]
            ca,cb = a.mean(axis=0),b.mean(axis=0)
            u,_,vt = np.linalg.svd((a-ca).T@(b-cb))
            rotation = vt.T@u.T
            if np.linalg.det(rotation)<0:
                vt[-1]*=-1
                rotation = vt.T@u.T
            translation = cb-rotation@ca
            delta = np.eye(4)
            delta[:3,:3],delta[:3,3] = rotation,translation
            transform = delta@transform
            if np.linalg.norm(translation)<.001 and np.linalg.norm(rotation-np.eye(3))<.001:
                break
    moved = source@transform[:3,:3].T+transform[:3,3]
    distances,_ = tree.query(moved,workers=1)
    valid = distances<stages[-1]
    rmse = float(np.sqrt(np.mean(distances[valid]**2))) if valid.any() else 1e6
    overlap = float(valid.mean())
    return transform, dict(accepted=overlap>=.60 and rmse<=.14,
                           rmse=rmse,overlap=overlap,points=len(source))
