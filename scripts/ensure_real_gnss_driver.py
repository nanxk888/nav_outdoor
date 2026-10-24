"""Linux-only GPS owner guard. Never terminate a driver owned by another launcher."""
import argparse
import fcntl
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import time
import yaml


def find_drivers(proc=Path('/proc')):
    matches = []
    for entry in proc.iterdir():
        if not entry.name.isdigit(): continue
        try:
            args = (entry/'cmdline').read_bytes().split(b'\0')
            if any(Path(os.fsdecode(a)).name == 'gnss_receiver.py' for a in args if a):
                matches.append(int(entry.name))
        except (OSError, ValueError): continue
    return matches


def get_port(config):
    data = yaml.safe_load(config.read_text())
    port = data['lc29h_gnss']['ros__parameters']['port']
    if not isinstance(port, str) or not port.startswith('/dev/'):
        raise ValueError('GNSS port must be an absolute /dev path')
    return Path(port)


def device_owners(port, proc=Path('/proc')):
    device = port.stat()
    if not stat.S_ISCHR(device.st_mode):
        raise ValueError('Configured GPS path is not a character device')
    owners = []
    for entry in proc.iterdir():
        if not entry.name.isdigit(): continue
        try:
            for descriptor in (entry/'fd').iterdir():
                try:
                    target = descriptor.stat()
                    if stat.S_ISCHR(target.st_mode) and target.st_rdev == device.st_rdev:
                        owners.append(int(entry.name)); break
                except OSError: continue
        except OSError: continue
    return owners


def driver_command(root, config):
    return ['bash', str(root/'scripts/system_exec.sh'), sys.executable,
            str(root/'scripts/gnss_receiver.py'), '--ros-args', '--log-level', 'error',
            '--params-file', str(config), '-r', '__node:=lc29h_gnss',
            '-r', 'fix:=/hardware/gnss/fix',
            '-r', 'time_reference:=/hardware/gnss/receiver_time_unqualified']


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--cyclonedds', required=True)
    args = parser.parse_args()
    root = Path(args.root).resolve(); config = Path(args.config).resolve()
    xml = Path(args.cyclonedds).resolve()
    for file in (root/'scripts/gnss_receiver.py', root/'scripts/system_exec.sh', config, xml):
        if not file.is_file(): raise RuntimeError('GPS prerequisite missing: '+str(file))
    port = get_port(config)
    stopped = False; child = None
    def shutdown(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, shutdown); signal.signal(signal.SIGTERM, shutdown)
    # Serialize our own launchers only; external managers must not race to own the UART.
    runtime = Path('/tmp') / ('nxk-gnss-guard-'+str(os.getuid())+'.lock')
    fd = os.open(str(runtime), os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'w') as guard:
        try: fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError: raise RuntimeError('Another nxk GPS guard is running')
        missing_reported = False; busy_reported = False
        try:
            while not stopped:
                if child is not None:
                    code = child.poll()
                    if code is not None:
                        raise RuntimeError('Owned GPS driver exited with code '+str(code)+'; inspect error, no automatic restart')
                elif not find_drivers():
                    if not port.exists():
                        if not missing_reported:
                            print('GPS device unavailable; waiting for '+str(port), file=sys.stderr, flush=True)
                            missing_reported = True
                    else:
                        if device_owners(port):
                            if not busy_reported:
                                print('GPS serial device is already open; refusing duplicate driver', file=sys.stderr, flush=True)
                                busy_reported = True
                            time.sleep(.5)
                            continue
                        env = os.environ.copy()
                        for name in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','ROS_DISCOVERY_SERVER'):
                            env.pop(name, None)
                        env.update(ROS_DOMAIN_ID='96', ROS_LOCALHOST_ONLY='0',
                                   RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', CYCLONEDDS_URI=xml.as_uri())
                        # Recheck immediately before spawn, to reduce races with a manual launcher.
                        if not find_drivers():
                            child = subprocess.Popen(driver_command(root, config), cwd=root, env=env,
                                                     start_new_session=True)
                # Existing driver is left untouched. A live process is not proof of a valid fix;
                # GPS gate/web independently expose missing/stale/no-fix input.
                time.sleep(.5)
        finally:
            if child is not None and child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try: child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    try: child.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL); child.wait()


if __name__ == '__main__':
    try: main()
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print('GPS driver guard: '+str(error), file=sys.stderr)
        sys.exit(1)
