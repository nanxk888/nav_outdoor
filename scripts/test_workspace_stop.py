import unittest
from pathlib import PurePosixPath
import shutil
import subprocess
import sys
import tempfile
import time
from stop_real_workspace import targets


class ScopeTest(unittest.TestCase):
    def test_exact_checkout_and_descendants_only(self):
        root=PurePosixPath('/workspace/nxk')
        def p(parent,*args):return (parent,'123',list(args))
        table={1:p(0,'ros2','launch','d1h_nav2','real_workspace.launch.py','workspace:=/workspace/nxk'),
               2:p(1,'/opt/ros/humble/lib/nav2_planner/planner_server'),
               3:p(0,'python3','/other/scripts/navigation_web.py'),
               4:p(0,'/workspace/ws_livox/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node'),
               5:p(0,'python3','/workspace/nxk/scripts/ensure_real_gnss_driver.py'),
               6:p(5,'python3','/workspace/d1h-autonomy-final/scripts/gnss_receiver.py'),
               7:p(0,'python3','/workspace/d1h-autonomy-final/scripts/gnss_receiver.py'),
               8:p(0,'/workspace/nxk/install/d1h_fast_lio/lib/d1h_fast_lio/lio_odometry'),
               9:p(0,'/workspace/nxk-other/install/d1h_fast_lio/lib/d1h_fast_lio/lio_odometry')}
        self.assertEqual(set(targets(table,root)),{1,2,5,6,8})

    @unittest.skipUnless(sys.platform=='linux','Linux signals and /proc')
    def test_duplicate_launch_refused_before_ros_start(self):
        import fcntl
        from pathlib import Path
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'scripts').mkdir();(root/'.local').mkdir()
            entry=root/'scripts/launch_real_workspace.sh'
            shutil.copyfile(Path(__file__).with_name(entry.name),entry)
            with (root/'.local/real_workspace.lock').open('w') as lock:
                fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                result=subprocess.run(['bash',str(entry)],capture_output=True,text=True,timeout=5)
                self.assertEqual(result.returncode,1)
                self.assertIn('Workspace already running',result.stderr)

    @unittest.skipUnless(sys.platform=='linux','Linux signals and /proc')
    def test_stop_scoped_live_child(self):
        from pathlib import Path
        with tempfile.TemporaryDirectory() as directory:
            scripts=Path(directory)/'scripts';scripts.mkdir()
            shutil.copyfile(Path(__file__).with_name('stop_real_workspace.py'),scripts/'stop_real_workspace.py')
            fixture=scripts/'navigation_web.py'
            fixture.write_text('import time\ntime.sleep(30)\n')
            child=subprocess.Popen([sys.executable,str(fixture)],start_new_session=True)
            unrelated=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'],start_new_session=True)
            try:
                time.sleep(.1)
                preview=subprocess.check_output([sys.executable,str(scripts/'stop_real_workspace.py'),'--dry-run'],text=True)
                self.assertIn('STOP '+str(child.pid)+' ',preview)
                self.assertNotIn('STOP '+str(unrelated.pid)+' ',preview)
                subprocess.run([sys.executable,str(scripts/'stop_real_workspace.py')],check=True,timeout=20)
                child.wait(timeout=2)
                self.assertIsNone(unrelated.poll())
            finally:
                for p in (child,unrelated):
                    if p.poll() is None:p.terminate()
                    p.wait()


if __name__=='__main__':unittest.main()
