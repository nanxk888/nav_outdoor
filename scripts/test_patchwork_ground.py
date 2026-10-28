"""Official native binding tests, synthetic scenes only; no robot."""
import sys,unittest,time
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root/'.local/patchworkpp'),str(root/'src/d1h_nav2')]
from d1h_nav2.patchwork_ground import PatchworkGround

class PatchworkTests(unittest.TestCase):
    def scene(self):
        x,y=np.meshgrid(np.arange(-5.5,5.5,.07),np.arange(-5.5,5.5,.07))
        floor=np.c_[x.ravel(),y.ravel(),np.full(x.size,-.46)]
        y,z=np.meshgrid(np.arange(-.5,.5,.03),np.arange(-.3,.4,.03))
        wall=np.c_[np.full(y.size,1.5),y.ravel(),z.ravel()]
        return floor,wall
    def test_ground_clutter_height_and_world_translation(self):
        floor,wall=self.scene();engine=PatchworkGround(.46)
        for origin in [(0,0,0),(0,0,.2),(0,0,.35)]:
            nav,clear,plane=engine.split(np.r_[floor,wall],origin)
            self.assertGreater(len(nav),len(wall)*.7)
            self.assertLess(np.count_nonzero(np.abs(nav[:,2]+.46)<.01),10)
            self.assertGreater(len(clear),1000)
            self.assertAlmostEqual(plane[2],-.46,places=2)
        translated=PatchworkGround(.46);offset=np.array([10.,-5.,2.])
        nav,clear,plane=translated.split(np.r_[floor,wall]+offset,offset)
        self.assertAlmostEqual(plane[2],1.54,places=2)
        print('native Patchwork++ diagnostics:',translated.diagnostics)
    def test_low_obstacle_not_ground(self):
        floor,_=self.scene();x,y=np.meshgrid(np.arange(.9,1.3,.025),np.arange(.9,1.3,.025))
        ledge=np.c_[x.ravel(),y.ravel(),np.full(x.size,-.41)]
        engine=PatchworkGround(.46);nav,_,_=engine.split(np.r_[floor,ledge],(0,0,0))
        self.assertGreater(np.count_nonzero(np.abs(nav[:,2]+.41)<.005),len(ledge)*.7)
    def test_no_ground_rejects(self):
        _,wall=self.scene()
        with self.assertRaises(ValueError):PatchworkGround(.46).split(wall,(0,0,0))
    def test_moderate_slope(self):
        floor,_=self.scene();floor[:,2]+=.08*floor[:,0]
        nav,clear,_=PatchworkGround(.46).split(floor,(0,0,0))
        self.assertLess(len(nav),len(floor)*.1);self.assertGreater(len(clear),1000)
    def test_clutter_and_rotated_scene(self):
        floor,wall=self.scene();floor=floor[::3]
        clutter=[]
        for angle in np.arange(0,2*np.pi,np.pi/4):
            c,s=np.cos(angle),np.sin(angle);r=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
            clutter.append(wall@r.T)
        points=np.vstack([floor]+clutter)
        for angle in (0.,.7,2.4):
            c,s=np.cos(angle),np.sin(angle);r=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
            nav,clear,_=PatchworkGround(.46).split(points@r.T,(0,0,0))
            self.assertGreater(len(nav),2000);self.assertGreater(len(clear),1000)

if __name__=='__main__':unittest.main()
