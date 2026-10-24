"""Fixture-only guard tests: no USB, no driver spawn."""
import os
from pathlib import Path
import tempfile
import sys
import types
import unittest
if os.name == 'nt': sys.modules.setdefault('fcntl', types.ModuleType('fcntl'))
from ensure_real_gnss_driver import find_drivers, get_port, driver_command, device_owners


class GuardTests(unittest.TestCase):
    def test_detection(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'123').mkdir();(root/'124').mkdir()
            (root/'123/cmdline').write_bytes(b'python3\0/old/scripts/gnss_receiver.py\0--ros-args\0')
            (root/'124/cmdline').write_bytes(b'python3\0/real_gnss\0lc29h_gnss\0')
            self.assertEqual(find_drivers(root),[123])

    def test_disappearing_process(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'125').mkdir()
            self.assertEqual(find_drivers(root),[])

    def test_port(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'gnss.yaml'
            p.write_text('lc29h_gnss:\n  ros__parameters:\n    port: /dev/serial/by-id/test\n')
            self.assertEqual(str(get_port(p)).replace('\\','/'),'/dev/serial/by-id/test')

    def test_reject_nondevice_path(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'gnss.yaml'
            p.write_text('lc29h_gnss:\n  ros__parameters:\n    port: /tmp/wrong\n')
            with self.assertRaises(ValueError):get_port(p)

    def test_command(self):
        command=driver_command(Path('/old'),Path('/private/gnss.yaml'))
        self.assertEqual(command[0],'bash')
        self.assertIn('fix:=/hardware/gnss/fix',command)
        self.assertIn(str(Path('/private/gnss.yaml')),command)

    @unittest.skipIf(os.name == 'nt', 'Linux /proc and pseudo-terminal check')
    def test_open_device_detected(self):
        import pty
        master, slave = pty.openpty()
        try:
            self.assertIn(os.getpid(), device_owners(Path(os.ttyname(slave))))
        finally:
            os.close(slave); os.close(master)


if __name__=='__main__':unittest.main()
