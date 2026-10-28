"""ROS-independent, conservative OSM road graph for the real robot only."""
import heapq
import math
import xml.etree.ElementTree as ET


def project(lat, lon, lat0, lon0):
    # Same local tangent approximation as gps_goal_to_enu; map must use it too.
    return (6378137.0 * math.radians(lon-lon0) * math.cos(math.radians(lat0)),
            6378137.0 * math.radians(lat-lat0))


def sample(a, b, step=.2):
    n = max(1, math.ceil(math.dist(a, b)/step))
    return [(a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n)
            for i in range(n+1)]


def nearest(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return (a[0]+t*dx, a[1]+t*dy)


class RoadGraph:
    def __init__(self, filename, lat0, lon0, width=3., way_ids=None, campus_id='287806713'):
        if not math.isfinite(width) or width <= .8:
            raise ValueError('Road corridor width must be finite and >0.8m; this is geometry, not an approval flag')
        root = ET.parse(filename).getroot()
        self.width = width
        self.xy = {n.get('id'): project(float(n.get('lat')), float(n.get('lon')), lat0, lon0)
                   for n in root.findall('node')}
        ways = {w.get('id'): w for w in root.findall('way')}
        polygon = [self.xy[n.get('ref')] for n in ways[campus_id].findall('nd')]
        def inside(p):
            x, y = p; result = False
            for (a,b),(c,d) in zip(polygon, polygon[1:]+polygon[:1]):
                if (b>y)!=(d>y) and x < (c-a)*(y-b)/(d-b)+a:
                    result = not result
            return result
        self.contains=inside
        self.edges = []
        automatic=not way_ids
        for wid in (way_ids or list(ways)):
            w = ways[str(wid)]
            tags = {t.get('k'):t.get('v') for t in w.findall('tag')}
            if (tags.get('highway') not in {'service','footway','path','pedestrian','living_street','residential'}
                    or tags.get('access') in {'no','private'} or tags.get('foot') == 'no'
                    or tags.get('bridge') == 'yes' or tags.get('tunnel') == 'yes'
                    or tags.get('layer','0') != '0'):
                if automatic:continue
                raise ValueError('Unsupported/restricted way: '+str(wid))
            ids = [n.get('ref') for n in w.findall('nd')]
            for a,b in zip(ids, ids[1:]):
                if a != b and self.xy[a] != self.xy[b] and inside(self.xy[a]) and inside(self.xy[b]):
                    # Conservative: honor vehicle oneway even on robot routes.
                    direction = tags.get('oneway','no')
                    self.edges.append((a,b,direction))
        if not self.edges:
            raise ValueError('OSM contains no supported ground-level roads within the deployment boundary')

    def distance(self, p):
        return min(math.dist(p, nearest(p,self.xy[a],self.xy[b])) for a,b,_ in self.edges)

    def route(self, start, goal, clear=lambda a,b: True, snap_limit=2.,
              offroad_start_limit=0., connector_clear=None):
        if not self.contains(start) or not self.contains(goal):raise ValueError('Start/goal outside deployment campus boundary')
        # Attach to road SEGMENTS, not just vertices. No free-space shortcut.
        attachments = []
        for index,p in enumerate((start,goal)):
            candidates = [(math.dist(p,q),i,q) for i,(a,b,_) in enumerate(self.edges)
                          for q in [nearest(p,self.xy[a],self.xy[b])]]
            d,i,q = min(candidates)
            in_corridor=d <= min(snap_limit,self.width/2-.25)
            if not in_corridor:
                if index!=0 or not 0<offroad_start_limit<=30 or d>offroad_start_limit or connector_clear is None or not connector_clear(p,q):
                    raise ValueError('Off-road start needs bounded observed-free access; off-road goal remains unsupported')
                if not all(self.contains(v) for v in sample(p,q)):raise ValueError('Connector leaves deployment campus boundary')
            elif not clear(p,q):
                raise ValueError('Start/goal connector blocked')
            attachments.append((i,q))
        xy = dict(self.xy); xy['S']=start;xy['G']=goal
        graph = {}
        def link(a,b):
            if clear(xy[a],xy[b]):graph.setdefault(a,[]).append((b,math.dist(xy[a],xy[b])))
        for i,(a,b,direction) in enumerate(self.edges):
            inserts = [(math.dist(xy[a],q),name,q) for name,(idx,q) in zip(('P','Q'),attachments) if idx==i]
            chain=[a]
            for _,name,q in sorted(inserts):xy[name]=q;chain.append(name)
            chain.append(b)
            for u,v in zip(chain,chain[1:]):
                if direction!='-1':link(u,v)
                if direction not in {'yes','1','true'}:link(v,u)
        # Do not snap/teleport the robot pose. Keep the true start in the path.
        if self.distance(start)>min(snap_limit,self.width/2-.25):
            graph.setdefault('S',[]).append(('P',math.dist(start,xy['P'])))
        else:link('S','P');link('P','S')
        link('Q','G');link('G','Q')
        queue=[(math.dist(start,goal),0.,'S')]; costs={'S':0.}; parent={}
        while queue:
            _,cost,u=heapq.heappop(queue)
            if cost != costs[u]:continue
            if u=='G':
                chain=[u]
                while chain[-1]!='S':chain.append(parent[chain[-1]])
                return [xy[n] for n in reversed(chain)]
            for v,d in graph.get(u,[]):
                new=cost+d
                if new < costs.get(v,math.inf):
                    costs[v]=new;parent[v]=u
                    heapq.heappush(queue,(new+math.dist(xy[v],goal),new,v))
        raise ValueError('No connected, obstacle-free road route')
