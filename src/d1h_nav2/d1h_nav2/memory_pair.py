"""Bounded source-stamped join; unrelated latest messages never replace a pair."""
class MemoryPair:
    def __init__(self):
        self.pending={'grid':{},'memory':{}};self.last=-1

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
