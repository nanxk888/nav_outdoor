"""Offline road routing regression: no ROS, simulator, hardware or network needed."""
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'src'/'d1h_nav2'))
from d1h_nav2.osm_roads import RoadGraph, project


class RoadsTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.file=pathlib.Path(self.tmp.name)/'map.osm'
        nodes={1:(0,0),2:(.0001,0),3:(.0001,.0001),4:(0,.0001),
               10:(-.001,-.001),11:(.001,-.001),12:(.001,.001),13:(-.001,.001)}
        text='<osm>'+''.join(f'<node id="{i}" lon="{x}" lat="{y}"/>' for i,(x,y) in nodes.items())
        for wid,ids in [(287806713,[10,11,12,13,10]),(20,[1,2,3]),(21,[1,4,3])]:
            text+=f'<way id="{wid}">'+''.join(f'<nd ref="{i}"/>' for i in ids)
            if wid!=287806713:text+='<tag k="highway" v="footway"/>'
            text+='</way>'
        self.file.write_text(text+'</osm>')
        self.graph=RoadGraph(self.file,0,0,4.,[20,21])

    def tearDown(self):self.tmp.cleanup()

    def test_segment_attachment_and_no_diagonal_shortcut(self):
        route=self.graph.route((3.,0.),(11.131949079,8.))
        self.assertTrue(any(abs(x-11.131949079)<.001 and abs(y)<.001 for x,y in route))

    def test_obstacle_reroutes_other_branch(self):
        route=self.graph.route((0.,0.),(11.131949079,11.131949079),
                               lambda a,b:not(a[1]<1 and b[1]<1 and max(a[0],b[0])>5))
        self.assertTrue(any(y>10 and x<1 for x,y in route))

    def test_offroad_goal_rejected(self):
        with self.assertRaises(ValueError):self.graph.route((0.,0.),(5.,5.))

    def test_offroad_start_nearest_connector_requires_evidence(self):
        start=(5.,-4.)
        with self.assertRaises(ValueError):self.graph.route(start,(8.,0.),offroad_start_limit=10.)
        with self.assertRaises(ValueError):self.graph.route(start,(8.,0.),offroad_start_limit=10.,connector_clear=lambda a,b:False)
        route=self.graph.route(start,(8.,0.),offroad_start_limit=10.,connector_clear=lambda a,b:True)
        self.assertEqual(route[0],start);self.assertEqual(route[1],(5.,0.))
        with self.assertRaises(ValueError):self.graph.route(start,(8.,0.),offroad_start_limit=2.,connector_clear=lambda a,b:True)

    def test_no_route_does_not_fallback_to_free_space(self):
        with self.assertRaises(ValueError):self.graph.route((0.,0.),(11.,11.),lambda a,b:a==b)

    def test_missing_review_rejected(self):
        with self.assertRaises(ValueError):RoadGraph(self.file,0,0,4.,[])

    def test_projection_matches_enu(self):
        self.assertAlmostEqual(project(.0001,.0001,0,0)[0],11.131949079,places=6)


if __name__=='__main__':unittest.main()
