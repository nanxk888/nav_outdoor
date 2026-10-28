"""Bounded source-stamped join; unrelated latest messages never replace a pair."""
class MemoryPair:
    def __init__(self):
        self.pending={'grid':{},'memory':{}};self.last=-1

    def diagnostics(self,now):
        def latest(kind):
            b=self.pending[kind]
            if not b:return None,None
            k=max(b);return k,now-b[k][1]
        g,gr=latest('grid');m,mr=latest('memory')
        missing=[rx for kind in self.pending for k,(_,rx) in self.pending[kind].items()
                 if k>self.last and k not in self.pending['memory' if kind=='grid' else 'grid']]
        return dict(latest_grid_stamp_ms=g,latest_memory_stamp_ms=m,latest_grid_receive_age_sec=gr,
                    latest_memory_receive_age_sec=mr,pair_wait_sec=max(0.,now-min(missing)) if missing else 0.)

    def add(self,kind,stamp,value,received):
        key=round(stamp*1000)  # ROS JSON seconds lose nanosecond precision.
        if key<=0 or key<self.last:return None
        bucket=self.pending[kind];bucket[key]=(value,received)
        for k in sorted(bucket)[:-6]:del bucket[k]
        common=set(self.pending['grid']) & set(self.pending['memory'])
        if not common:return None
        key=max(common)
        if key<=self.last:return None
        self.last=key
        g,gr=self.pending['grid'][key];m,mr=self.pending['memory'][key]
        return g,m,gr,mr
