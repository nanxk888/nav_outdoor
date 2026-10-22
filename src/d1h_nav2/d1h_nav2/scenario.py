"""Read prior geometry directly from the same SDF world Gazebo loads."""
import math
import xml.etree.ElementTree as ET


def known_boxes(world_path):
    boxes = []
    for model in ET.parse(world_path).getroot().findall('./world/model'):
        if not model.get('name', '').startswith('known_'):
            continue
        pose = [float(v) for v in model.findtext('pose', '0 0 0 0 0 0').split()]
        size = model.findtext('./link/collision/geometry/box/size')
        if size is None or any(abs(a) > 1e-8 for a in pose[3:5]):
            raise ValueError('Prior obstacles must be upright collision boxes')
        sx, sy, sz = map(float, size.split())
        boxes.append((model.get('name'), pose[0], pose[1], pose[2], sx, sy, sz, pose[5]))
    return boxes


def rasterize_boxes(data, width, resolution, origin_x, origin_y, boxes, clearance_m=0.9):
    for _, cx, cy, cz, sx, sy, sz, yaw in boxes:
        # Ignore roofs above the conservative standing envelope in 2D only.
        if cz-sz/2 >= clearance_m or cz+sz/2 <= 0.03:
            continue
        c, s = math.cos(yaw), math.sin(yaw)
        ex, ey = abs(c)*sx/2 + abs(s)*sy/2, abs(s)*sx/2 + abs(c)*sy/2
        x0 = max(0, math.floor((cx-ex-origin_x)/resolution))
        x1 = min(width-1, math.ceil((cx+ex-origin_x)/resolution)-1)
        y0 = max(0, math.floor((cy-ey-origin_y)/resolution))
        y1 = min(width-1, math.ceil((cy+ey-origin_y)/resolution)-1)
        # Conservative cell/rectangle overlap via the four separating axes.
        half = resolution/2
        for y in range(y0, y1+1):
            for x in range(x0, x1+1):
                dx = origin_x+(x+0.5)*resolution-cx
                dy = origin_y+(y+0.5)*resolution-cy
                # Boundary-only contact has zero area; do not add another
                # occupied row/column around grid-aligned physical boxes.
                if (abs(dx) < ex+half-1e-9 and abs(dy) < ey+half-1e-9
                        and abs(c*dx+s*dy) < sx/2+half*(abs(c)+abs(s))-1e-9
                        and abs(-s*dx+c*dy) < sy/2+half*(abs(c)+abs(s))-1e-9):
                    data[y*width+x] = 100
