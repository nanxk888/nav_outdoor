"""Bounded LIO-frame observation memory and candidate-only known-free A*."""
import heapq
import math
import time
from d1h_nav2.footprint import circle_cells,path_free,body_cells,body_path_free


class ObservedMemory:
    def __init__(self,resolution=.25,retention=300.,max_cells=100000,radius=.25,min_points=1,confirm_frames=1):
        if not .1<=resolution<=1 or not (retention==0 or 1<=retention<=600) or not (max_cells==0 or 100<=max_cells<=100000) or not .25<=radius<=1.5:
            raise ValueError('Invalid bounded memory parameters')
        self.resolution=resolution;self.retention=retention;self.max_cells=max_cells;self.radius=radius
        self.cells={};self.time=None
        self.min_points=min_points;self.confirm_frames=confirm_frames;self.hits={};self.free_hits={}
        self.body_only=set();self.heading=None

    def cell(self,p):return (math.floor(p[0]/self.resolution),math.floor(p[1]/self.resolution))
    def point(self,c):return ((c[0]+.5)*self.resolution,(c[1]+.5)*self.resolution)
    def expire(self,now):
        if self.time is not None and now<self.time:self.cells.clear()
        self.time=now
        # Zero means session lifetime. Avoid rebuilding the entire map every tick.
        if self.retention:
            self.cells={c:v for c,v in self.cells.items() if 0<=now-v[1]<self.retention}

    def observe(self,origin,obstacles,returns,now,max_range=6.,heading=None,rear_blind_deg=120.):
        """Observed rays only; no return is NOT free. All obstacles dominate this scan.

        This is planar line-of-sight evidence, NOT terrain/pit traversability proof.
        Ray budgets are bounded separately from sparse stored cells.
        """
        for c in self.body_only:self.cells.pop(c,None)
        self.body_only.clear();self.heading=heading
        self.expire(now)
        def blind(p):
            if heading is None:return False
            a=math.atan2(p[1]-origin[1],p[0]-origin[0])-heading-math.pi
            return abs(math.atan2(math.sin(a),math.cos(a)))<=math.radians(rear_blind_deg/2)
        def allowed(p):return all(math.isfinite(v) for v in p[:2]) and math.dist(origin[:2],p[:2])<=max_range
        counts={}
        for p in obstacles:
            if allowed(p):
                c=self.cell(p);counts[c]=counts.get(c,0)+1
        raw=set(counts)
        self.hits={c:(self.hits.get(c,0)+1) for c in raw}
        occupied={c for c in raw if self.hits[c]>=self.confirm_frames or
                  sum(counts.get((c[0]+i,c[1]+j),0) for i in (-1,0,1) for j in (-1,0,1))>=self.min_points}
        start=self.cell(origin)
        # Deduplicate before ray traversal, with deterministic bounded endpoint sampling.
        endpoints=sorted({self.cell(p) for p in returns if allowed(p) and not blind(p)})
        cleared=set()
        stride=max(1,math.ceil(len(endpoints)/2000))
        for end in endpoints[::stride]:
            count=max(abs(end[0]-start[0]),abs(end[1]-start[1]),1)
            for i in range(count+1):
                c=(round(start[0]+(end[0]-start[0])*i/count),round(start[1]+(end[1]-start[1])*i/count))
                if c in raw:break
                if blind(self.point(c)):continue
                if c in cleared:continue
                cleared.add(c)
                if self.cells.get(c,(0,0))[0]==100:
                    self.free_hits[c]=self.free_hits.get(c,0)+1
                    if self.free_hits[c]<2:continue
                self.cells[c]=(0,now)
        for c in occupied:self.cells[c]=(100,now);self.free_hits.pop(c,None)
        # Transient current-body support, never an observed-free trail or obstacle eraser.
        if heading is not None:
            for c in body_cells(origin,self.resolution,heading):
                if c not in self.cells and c not in raw:
                    self.cells[c]=(0,now);self.body_only.add(c)
        self.free_hits={c:n for c,n in self.free_hits.items() if c in cleared and self.cells.get(c,(0,0))[0]==100}
        if self.max_cells and len(self.cells)>self.max_cells:
            self.cells=dict(sorted(self.cells.items(),key=lambda item:item[1][1],reverse=True)[:self.max_cells])

    def route(self,start,goal,now,explore=True,max_expand=20000):
        self.expire(now);s=self.cell(start);g=self.cell(goal)
        r=math.ceil(self.radius/self.resolution)
        cells=lambda p:body_cells(p,self.resolution,self.heading) if self.heading is not None else circle_cells(p,self.resolution,self.radius)
        def position(c):
            return tuple(start[:2]) if c==s else (tuple(goal[:2]) if c==g else self.point(c))
        def segment_free(a,b):
            return (body_path_free([a,b],self.resolution,lambda c:self.cells.get(c,(-1,0))[0]==0) if self.heading is not None else path_free([a,b],self.resolution,self.radius,lambda c:self.cells.get(c,(-1,0))[0]==0))
        cache={}
        def free(c):
            if c not in cache:
                cache[c]=all(self.cells.get(k,(-1,0))[0]==0 for k in cells(position(c)))
            return cache[c]
        bad=[(c,self.cells.get(c,(-1,0))[0]) for c in cells(start)
             if self.cells.get(c,(-1,0))[0]!=0]
        if bad:raise ValueError('Start footprint blocked: radius=%.2fm unknown=%d occupied=%d cells=%s' %
                               (self.radius,sum(v<0 for _,v in bad),sum(v>0 for _,v in bad),bad))
        queue=[(math.dist(s,g),0.,s)];costs={s:0.};parent={};best=None;best_score=math.inf
        count=0;reached=False;deadline=time.monotonic()+.2
        while queue and count<max_expand:
            _,cost,c=heapq.heappop(queue)
            if cost!=costs[c]:continue
            count+=1
            if count%64==0 and time.monotonic()>deadline:raise ValueError('Planning time budget exceeded; no candidate released')
            if c==g:best=c;reached=True;break
            # Frontier standoff keeps the entire circular footprint in known-free cells.
            frontier=any((c[0]+dx,c[1]+dy) not in self.cells for dx,dy in ((r+2,0),(-r-2,0),(0,r+2),(0,-r-2)))
            if explore and frontier and math.dist(c,g)<math.dist(s,g)-1:
                score=math.dist(c,g)+.15*cost
                if score<best_score:best=c;best_score=score
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                n=(c[0]+dx,c[1]+dy)
                if not free(n):continue
                if dx and dy and (not free((c[0]+dx,c[1])) or not free((c[0],c[1]+dy))):continue
                if (c==s or n==g) and not segment_free(position(c),position(n)):continue
                new=cost+math.hypot(dx,dy)
                if new<costs.get(n,math.inf):
                    costs[n]=new;parent[n]=c;heapq.heappush(queue,(new+math.dist(n,g),new,n))
        if count>=max_expand and not reached:raise ValueError('Planning budget exceeded; no candidate released')
        if best is None:raise ValueError('No known-free route or goal-directed frontier; do not enter unknown space')
        chain=[best]
        while chain[-1]!=s:chain.append(parent[chain[-1]])
        if len(chain)>4000:raise ValueError('Path point budget exceeded; no candidate released')
        points=[position(c) for c in reversed(chain)]
        # Preserve actual endpoints, rather than stopping at a quantized cell centre.
        points=[tuple(start[:2])]+points+([tuple(goal[:2])] if reached else [])
        def segment_free(a,b):
            return (body_path_free([a,b],self.resolution,lambda c:self.cells.get(c,(-1,0))[0]==0) if self.heading is not None else path_free([a,b],self.resolution,self.radius,lambda c:self.cells.get(c,(-1,0))[0]==0))
        def valid(path):return all(segment_free(a,b) for a,b in zip(path,path[1:]))
        if not valid(points):
            for index,(a,b) in enumerate(zip(points,points[1:])):
                if not segment_free(a,b):
                    raise ValueError(f'Path footprint blocked/unknown: segment={index} from={a} to={b} radius={self.radius}')
        # Remove grid stair-steps locally, then round corners (Chaikin). Never
        # release a smoothed shortcut unless its swept footprint remains free.
        simplified=[points[0]];index=0
        while index<len(points)-1:
            end=min(index+12,len(points)-1)
            while end>index+1 and not segment_free(points[index],points[end]):end-=1
            simplified.append(points[end]);index=end
        rounded=simplified
        for _ in range(2):
            candidate=[rounded[0]]
            for a,b in zip(rounded,rounded[1:]):
                candidate.extend([(a[0]*.75+b[0]*.25,a[1]*.75+b[1]*.25),
                                  (a[0]*.25+b[0]*.75,a[1]*.25+b[1]*.75)])
            candidate.append(rounded[-1])
            if valid(candidate):rounded=candidate
            else:break
        # Dense samples keep nearest-point tracking meaningful on straight legs.
        dense=[rounded[0]]
        for a,b in zip(rounded,rounded[1:]):
            steps=max(1,math.ceil(math.dist(a,b)/(self.resolution/2)))
            dense.extend([(a[0]+(b[0]-a[0])*k/steps,a[1]+(b[1]-a[1])*k/steps)
                          for k in range(1,steps+1)])
        dense[-1]=rounded[-1]
        if len(dense)>4000:raise ValueError('Smoothed path point budget exceeded')
        return {'points':dense,'partial':not reached,'smoothing':'validated_corner_rounding',
                'reason':'known-free target route' if reached else 'frontier standoff only; goal remains unobserved/unreachable',
                'expanded':count,'candidate_only':True}
