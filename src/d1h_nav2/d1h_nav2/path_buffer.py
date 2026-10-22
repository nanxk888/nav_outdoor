"""Geometry helpers for collision-checked path handover, not obstacle smoothing."""
import copy
import math


def length(path):
    return sum(math.hypot(b.pose.position.x-a.pose.position.x,
                          b.pose.position.y-a.pose.position.y)
               for a, b in zip(path.poses, path.poses[1:]))


def prune_and_sample(path, x, y, step=.05):
    out = copy.deepcopy(path)
    if len(path.poses) < 2:
        distance = math.inf if not path.poses else math.hypot(
            path.poses[0].pose.position.x-x, path.poses[0].pose.position.y-y)
        return out, distance
    best = (math.inf, 0, 0.)
    for i, (a, b) in enumerate(zip(path.poses, path.poses[1:])):
        ax, ay = a.pose.position.x, a.pose.position.y
        dx, dy = b.pose.position.x-ax, b.pose.position.y-ay
        t = max(0., min(1., ((x-ax)*dx+(y-ay)*dy)/max(dx*dx+dy*dy, 1e-12)))
        distance = math.hypot(ax+t*dx-x, ay+t*dy-y)
        if distance < best[0]:
            best = (distance, i, t)
    distance, i, t = best
    first = copy.deepcopy(path.poses[i])
    a, b = path.poses[i].pose.position, path.poses[i+1].pose.position
    first.pose.position.x, first.pose.position.y = a.x+t*(b.x-a.x), a.y+t*(b.y-a.y)
    points = [first]+copy.deepcopy(path.poses[i+1:])
    out.poses = [first]
    for a, b in zip(points, points[1:]):
        d = math.hypot(b.pose.position.x-a.pose.position.x, b.pose.position.y-a.pose.position.y)
        for j in range(1, max(1, math.ceil(d/step))+1):
            f = j/max(1, math.ceil(d/step))
            p = copy.deepcopy(b)
            p.pose.position.x = a.pose.position.x+f*(b.pose.position.x-a.pose.position.x)
            p.pose.position.y = a.pose.position.y+f*(b.pose.position.y-a.pose.position.y)
            out.poses.append(p)
    return out, distance


def trim_end(path, margin):
    out = copy.deepcopy(path)
    removed = 0.
    while len(out.poses) > 1 and removed < margin:
        a, b = out.poses[-2].pose.position, out.poses[-1].pose.position
        removed += math.hypot(b.x-a.x, b.y-a.y)
        out.poses.pop()
    return out
