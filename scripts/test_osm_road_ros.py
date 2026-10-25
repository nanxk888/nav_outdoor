"""Isolated ROS action/interface smoke test. No simulated or real motors."""
import json
import os
import pathlib
import threading
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import ComputePathToPose
from test_osm_roads import RoadsTest
from d1h_nav2.osm_road_planner import OsmRoadPlanner
from nav_msgs.msg import OccupancyGrid
from array import array


def main():
    assert 80<=int(os.environ.get('ROS_DOMAIN_ID','-1'))<=99,'Use isolated ROS_DOMAIN_ID=89'
    fixture=RoadsTest();fixture.setUp()
    profile=pathlib.Path(fixture.tmp.name)/'profile.json'
    profile.write_text(json.dumps(dict(osm_file=str(fixture.file),
        datum_latitude_deg=0.,datum_longitude_deg=0.,road_width_m=4.)))
    rclpy.init(args=['--ros-args','-p','profile:='+str(profile)])
    node=OsmRoadPlanner();client_node=rclpy.create_node('road_action_test')
    client=ActionClient(client_node,ComputePathToPose,'/roads/compute_path_to_pose')
    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(node);executor.add_node(client_node)
    thread=threading.Thread(target=executor.spin);thread.start()
    def wait(future):
        event=threading.Event();future.add_done_callback(lambda _:event.set())
        assert event.wait(10),'Action timed out'
        return future.result()
    try:
        assert client.wait_for_server(timeout_sec=5)
        goal=ComputePathToPose.Goal();goal.use_start=True
        goal.start.header.frame_id=goal.goal.header.frame_id='map'
        goal.start.pose.orientation.w=goal.goal.pose.orientation.w=1.
        goal.goal.pose.position.x=11.131949079;goal.goal.pose.position.y=11.131949079
        # Missing live costmap must fail closed.
        result=wait(wait(client.send_goal_async(goal)).get_result_async())
        assert result.status==6 and not result.result.path.poses
        node.on_map(node.grid)
        result=wait(wait(client.send_goal_async(goal)).get_result_async())
        assert result.status==4 and len(result.result.path.poses)>20
        access=OccupancyGrid();access.header.frame_id='map';access.header.stamp=client_node.get_clock().now().to_msg()
        access.info.resolution=.25;access.info.width=100;access.info.height=100
        access.info.origin.position.x=-10.;access.info.origin.position.y=-10.;access.info.origin.orientation.w=1.
        access.data=array('b',[0]*10000);node.on_access(access)
        goal.start.pose.position.x=5.;goal.start.pose.position.y=-4.
        goal.goal.pose.position.x=8.;goal.goal.pose.position.y=0.
        result=wait(wait(client.send_goal_async(goal)).get_result_async())
        assert result.status==4 and result.result.path.poses[0].pose.position.y==-4.,'true offroad start must remain in route'
        assert node.access_segment is not None
        goal.goal.pose.position.x=5.;goal.goal.pose.position.y=5.
        result=wait(wait(client.send_goal_async(goal)).get_result_async())
        assert result.status==6 and not result.result.path.poses
        print('PASS: ROS road action + observed offroad start access; true start preserved; missing map/offroad goal rejected')
    finally:
        executor.shutdown();thread.join();client.destroy();client_node.destroy_node();node.destroy_node()
        rclpy.shutdown();fixture.tearDown()


if __name__=='__main__':main()
