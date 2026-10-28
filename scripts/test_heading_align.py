import sys,unittest,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
from d1h_nav2.heading_align import HeadingAlign
class AlignTests(unittest.TestCase):
    def test_latch_and_release(self):
        a=HeadingAlign();p=[(0,0),(-1,0)]
        self.assertTrue(a.update((0,0),0,p))
        self.assertTrue(a.update((0,0),math.radians(90),[(0,0),(1,0)]),'turn heading must stay latched')
        self.assertFalse(a.update((0,0),math.radians(170),p))
    def test_shortest_angle(self):
        a=HeadingAlign()
        self.assertFalse(a.update((0,0),math.radians(350),[(0,0),(1,0)]))
        self.assertFalse(a.update((0,0),math.radians(90),[(0,0),(1,0)]))
if __name__=='__main__':unittest.main()
