"""Bounded LIO-frame observation memory and candidate-only known-free A*."""
import heapq
import math
import time


class ObservedMemory:
    def __init__(self,resolution=.25,retention=300.,max_cells=60000,radius=.25):
        if not .1<=resolution<=1 or not 1<=retention<=600 or not 100<=max_cells<=100000 or not .25<=radius<=1.5:
            raise ValueError('Invalid bounded memory parameters')
        self.resolution=resolution;self.retention=retention;self.max_cells=max_cells;self.radius=radius
        self.cells={};self.time=None

    def cell(self,p):return (math.floor(p[0]/self.resolution),math.floor(p[1]/self.resolution))
    def point(self,c):return ((c[0]+.5)*self.resolution,(c[1]+.5)*self.resolution)
    def expire(self,now):
        if self.time is not None and now<self.time:self.cells.clear()
        self.time=now
        self.cells={c:v for c,v in self.cells.items() if 0<=now-v[1]<self.retention}

    def observe(self,origin,obstacles,returns,now,max_range=6.):
        """Observed rays only; no return is NOT free. All obstacles dominate this scan.

        This is planar line-of-sight evidence, NOT terrain/pit traversability proof.
        Ray budgets are bounded separately from sparse stored cells.
        """
        self.expire(now)
        def allowed(p):return all(math.isfinite(v) for v in p[:2]) and math.dist(origin[:2],p[:2])<=max_range
        occupied={self.cell(p) for p in obstacles if allowed(p)}
        start=self.cell(origin)
        # Deduplicate before ray traversal, with deterministic bounded endpoint sampling.
        endpoints=sorted({self.cell(p) for p in returns if allowed(p)})
        stride=max(1,math.ceil(len(endpoints)/2000))
        for end in endpoints[::stride]:
            count=max(abs(end[0]-start[0]),abs(end[1]-start[1]),1)
            for i in range(count+1):
                c=(round(start[0]+(end[0]-start[0])*i/count),round(start[1]+(end[1]-start[1])*i/count))
                if c in occupied:break
                self.cells[c]=(0,now)
        for c in occupied:self.cells[c]=(100,now)
        if len(self.cells)>self.max_cells:
            self.cells=dict(sorted(self.cells.items(),key=lambda item:item[1][1],reverse=True)[:self.max_cells])

    def route(self,start,goal,now,explore=True,max_expand=20000):
        self.expire(now);s=self.cell(start);g=self.cell(goal)
        r=math.ceil(self.radius/self.resolution)
        offsets=[(x,y) for x in range(-r,r+1) for y in range(-r,r+1)
                 if math.hypot(x,y)*self.resolution<=self.radius+self.resolution*.71]
        cache={}
        def free(c):
            if c not in cache:
                cache[c]=all(self.cells.get((c[0]+x,c[1]+y),(-1,0))[0]==0 for x,y in offsets)
            return cache[c]
        if not free(s):raise ValueError('Robot footprint is not fully observed free; stop and observe')
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
                new=cost+math.hypot(dx,dy)
                if new<costs.get(n,math.inf):
                    costs[n]=new;parent[n]=c;heapq.heappush(queue,(new+math.dist(n,g),new,n))
        if count>=max_expand and not reached:raise ValueError('Planning budget exceeded; no candidate released')
        if best is None:raise ValueError('No known-free route or goal-directed frontier; do not enter unknown space')
        chain=[best]
        while chain[-1]!=s:chain.append(parent[chain[-1]])
        if len(chain)>4000:raise ValueError('Path point budget exceeded; no candidate released')
        points=[self.point(c) for c in reversed(chain)]
        # Preserve actual endpoints, rather than stopping at a quantized cell centre.
        points=[tuple(start[:2])]+points+([tuple(goal[:2])] if reached else [])
        def segment_free(a,b):
            steps=max(1,math.ceil(math.dist(a,b)/(self.resolution/8)))
            return all(free(self.cell((a[0]+(b[0]-a[0])*k/steps,
                                       a[1]+(b[1]-a[1])*k/steps))) for k in range(steps+1))
        def valid(path):return all(segment_free(a,b) for a,b in zip(path,path[1:]))
        if not valid(points):raise ValueError('Exact endpoint footprint blocked or unknown')
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
