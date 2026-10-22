"""Bounded world-frame cluster memory; no raw scans are retained."""
from dataclasses import dataclass, field
import math


@dataclass
class Track:
    center: tuple
    anchor: tuple
    since: float
    seen: float
    cells: set = field(default_factory=set)
    remembered: set = field(default_factory=set)
    confirmed: float = -math.inf
    cell_since: dict = field(default_factory=dict)
    cell_confirmed: dict = field(default_factory=dict)


class ObstacleMemory:
    def __init__(self, resolution=.25, stable_sec=5., retention_sec=360.,
                 gap_sec=.6, motion_m=.15, max_tracks=256, max_cells=100000,
                 remember_all=False):
        if (min(resolution, retention_sec, gap_sec, motion_m, max_tracks, max_cells) <= 0
                or stable_sec < 0 or (not remember_all and stable_sec == 0)):
            raise ValueError('Memory durations, resolution, tolerances and capacities must be positive')
        self.resolution, self.stable_sec, self.retention_sec = resolution, stable_sec, retention_sec
        self.gap_sec, self.motion_m = gap_sec, motion_m
        self.max_tracks, self.max_cells = max_tracks, max_cells
        self.tracks, self.last_time = [], None
        self.remember_all, self.immediate_cells = remember_all, {}

    def expire(self, now):
        if self.last_time is not None and now < self.last_time:
            self.tracks.clear()  # Gazebo reset / localization epoch change.
            self.immediate_cells.clear()
        self.last_time = now
        if self.remember_all:
            self.immediate_cells = {c: t for c, t in self.immediate_cells.items()
                                    if now-t < self.retention_sec}
            return
        for t in self.tracks:
            t.remembered = {c for c in t.remembered
                            if now-t.cell_confirmed.get(c, t.confirmed) < self.retention_sec}
            t.cell_confirmed = {c: stamp for c, stamp in t.cell_confirmed.items() if c in t.remembered}
        self.tracks = [t for t in self.tracks if t.remembered or now-t.seen <= self.gap_sec]

    def update(self, clusters, now):
        """clusters: [(world centroid (x,y), occupied grid-cell set)].

        Association failures reset the five-second qualification. Movement is
        measured against an anchor, not just the previous frame, so cumulative
        slow drift is detected too. This is geometric evidence, not semantics.
        """
        self.expire(now)
        if self.remember_all:
            # No motion classification and no cluster-count cutoff: every
            # observed occupied cell earns its own independent TTL.
            for _, cells in clusters:
                self.immediate_cells.update({c: now for c in cells})
            if len(self.immediate_cells) > self.max_cells:
                newest = sorted(self.immediate_cells.items(), key=lambda item: item[1], reverse=True)
                self.immediate_cells = dict(newest[:self.max_cells])
            return
        available = set(range(len(self.tracks)))
        for center, cells in sorted(clusters, key=lambda g: len(g[1]), reverse=True)[:self.max_tracks]:
            if not cells:
                continue
            distances = [(math.dist(center, self.tracks[i].center), i) for i in available]
            distance, index = min(distances, default=(math.inf, -1))
            if distance < .75:
                available.remove(index)
                t = self.tracks[index]
                gap = now-t.seen > self.gap_sec
                moved = math.dist(center, t.anchor) > self.motion_m
                # A shape that changed substantially may be an object merging,
                # splitting or moving, rather than a stationary wall.
                overlap = len(cells & t.cells) / max(1, min(len(cells), len(t.cells)))
                changed = overlap < .5
                if gap or moved or changed:
                    t.anchor, t.since = center, now
                    t.cell_since.clear()
                    if moved or changed:
                        t.remembered.clear()
                        t.cell_confirmed.clear()
                        t.confirmed = -math.inf
                t.center, t.seen, t.cells = center, now, cells
            else:
                t = Track(center, center, now, now, cells)
                self.tracks.append(t)
            t.cell_since = {c: t.cell_since.get(c, now) for c in cells}
            if now-t.since >= self.stable_sec:
                qualified = {c for c in cells if now-t.cell_since[c] >= self.stable_sec}
                t.remembered.update(qualified)
                t.cell_confirmed.update({c: now for c in qualified})
                if qualified:
                    t.confirmed = now
        # Hard bounds cover BOTH candidates and promoted records. Evict oldest
        # unobserved records first, rather than accumulating a pointcloud log.
        self.tracks.sort(key=lambda t: t.seen, reverse=True)
        self.tracks = self.tracks[:self.max_tracks]
        total, bounded = 0, []
        for t in self.tracks:
            size = len(t.cells | t.remembered)
            if total+size <= self.max_cells:
                bounded.append(t)
                total += size
        self.tracks = bounded

    def occupied(self, now):
        self.expire(now)
        if self.remember_all:
            return set(self.immediate_cells)
        result = set()
        for t in self.tracks:
            result.update(t.remembered)
        return result


def cluster_points(points, resolution=.25, max_cells=100000):
    """8-connected 2D clusters, centroid from cell centers (bounded input)."""
    cells = set()
    for x, y in points:
        if math.isfinite(x) and math.isfinite(y):
            cells.add((math.floor(x/resolution), math.floor(y/resolution)))
        if len(cells) >= max_cells:
            break
    groups = []
    while cells:
        first = cells.pop()
        group, todo = {first}, [first]
        while todo:
            x, y = todo.pop()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbor = (x+dx, y+dy)
                    if neighbor in cells:
                        cells.remove(neighbor)
                        group.add(neighbor)
                        todo.append(neighbor)
        center = (sum(x+.5 for x, y in group)*resolution/len(group),
                  sum(y+.5 for x, y in group)*resolution/len(group))
        groups.append((center, group))
    return groups
