import sys,math,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
from d1h_nav2.footprint import body_cells,body_path_free

class BodyTests(unittest.TestCase):
    def test_asymmetric_bounds(self):
        cells=set(body_cells((0,0),.01,0))
        self.assertEqual((min(x for x,y in cells),max(x for x,y in cells)),(-25,12))
        self.assertEqual((min(y for x,y in cells),max(y for x,y in cells)),(-23,22))
    def test_rotation(self):
        cells=set(body_cells((0,0),.01,math.pi/2))
        self.assertEqual((min(y for x,y in cells),max(y for x,y in cells)),(-25,12))
    def test_obstacle_is_not_erased(self):
        self.assertFalse(body_path_free([(0,0),(1,0)],.1,lambda c:c!=(5,0)))

if __name__=='__main__':unittest.main()
