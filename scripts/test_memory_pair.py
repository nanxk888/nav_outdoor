import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/d1h_nav2'))
from d1h_nav2.memory_pair import MemoryPair

class PairTests(unittest.TestCase):
    def test_both_arrival_orders_and_no_cross_frame(self):
        p=MemoryPair()
        self.assertIsNone(p.add('grid',100.,'g1',1.))
        self.assertEqual(p.add('memory',100.,'m1',2.),('g1','m1',1.,2.))
        self.assertIsNone(p.add('grid',100.,'duplicate',99.),'duplicate must not refresh paired receive time')
        self.assertIsNone(p.add('grid',100.5,'g2',3.))
        self.assertAlmostEqual(p.diagnostics(3.2)['pair_wait_sec'],.2)
        self.assertEqual(p.add('memory',100.5,'m2',4.),('g2','m2',3.,4.))
        self.assertIsNone(p.add('memory',101.,'m3',5.))
        self.assertEqual(p.add('grid',101.,'g3',6.),('g3','m3',6.,5.))
        self.assertIsNone(p.add('grid',100.,'old',99.))
    def test_bounded_and_invalid(self):
        p=MemoryPair()
        self.assertIsNone(p.add('grid',0.,None,0.))
        for i in range(1,100):p.add('grid',float(i),i,i)
        self.assertLessEqual(len(p.pending['grid']),6)

if __name__=='__main__':unittest.main()
