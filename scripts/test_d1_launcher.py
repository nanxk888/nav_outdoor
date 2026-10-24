"""Linux-only supervisor checks with fake /proc and harmless sleep children."""
import tempfile
from pathlib import Path
import subprocess
import unittest
from d1_receiver_launcher import conflicting_processes,stop_children


class LauncherTests(unittest.TestCase):
    def test_only_actual_receiver_or_filter_matches(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            samples={101:b'python3\0/home/robot/nxk_receiver/robot_udp_receiver.py\0',
                     102:b'/home/robot/bridge/lib/d1h_robot_bridge/fsm_filter\0',
                     103:b'bash\0-c\0echo robot_udp_receiver.py\0'}
            for pid,args in samples.items():
                p=root/str(pid);p.mkdir();(p/'cmdline').write_bytes(args)
            self.assertEqual(set(conflicting_processes(root)),{'101','102'})

    def test_cleanup_only_owned_process_groups(self):
        owned=subprocess.Popen(['/bin/sleep','30'],start_new_session=True)
        unrelated=subprocess.Popen(['/bin/sleep','30'],start_new_session=True)
        try:
            stop_children([owned]);self.assertIsNotNone(owned.poll());self.assertIsNone(unrelated.poll())
        finally:
            if owned.poll() is None:owned.kill();owned.wait()
            unrelated.terminate();unrelated.wait()


if __name__=='__main__':unittest.main()
