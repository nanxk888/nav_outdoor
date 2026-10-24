"""Loopback fake receiver + real CLI sender. No ROS or physical robot."""
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from robot_policy import ReceiverPolicy
from robot_wire import encode,decode


class PairTests(unittest.TestCase):
    def run_client(self, action, authorized=True, fsm='idle', extra=()):
        key=b'fake-loopback-only-never-use-on-a-robot'
        policy=ReceiverPolicy(authorized,True);modes=[];end=threading.Event();failures=[]
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',0));sock.settimeout(.03)
        def server():
            current=fsm
            while not end.is_set():
                now=time.monotonic();policy.feedback(current,now);policy.interfaces(True,True,now);policy.expire(now)
                try:packet,addr=sock.recvfrom(4097)
                except socket.timeout:continue
                try:
                    data=decode(packet,key);error=''
                    try:policy.accept(data,time.monotonic())
                    except ValueError as e:error=str(e)
                    state=policy.status(time.monotonic());state.update(kind='status',time=time.time(),reply_to=data['seq'],command_accepted=not error,rejection=error)
                    sock.sendto(encode(state,key),addr)
                    _,mode=policy.tick(time.monotonic())
                    if mode:
                        modes.append(mode);current='idle' if mode=='transform_down' else mode
                except Exception as e:failures.append(str(e));end.set()
        thread=threading.Thread(target=server);thread.start()
        try:
            with tempfile.TemporaryDirectory() as temp:
                secret=Path(temp)/'fixture.key';secret.write_bytes(key)
                result=subprocess.run([sys.executable,str(Path(__file__).with_name('robot_udp_test.py')),
                    '--bind','127.0.0.1','--robot','127.0.0.1','--port',str(sock.getsockname()[1]),
                    '--key-file',str(secret),'--action',action,'--duration','1',*extra],capture_output=True,text=True,timeout=5)
        finally:end.set();thread.join();sock.close()
        self.assertFalse(failures,failures)
        return result,policy,modes

    def test_readonly_status(self):
        r,p,m=self.run_client('status',authorized=False)
        self.assertEqual(r.returncode,0,r.stderr);self.assertFalse(p.active);self.assertEqual(m,[])
    def test_stand_and_exit_stop(self):
        r,p,m=self.run_client('stand',extra=['--authorize-motion'])
        self.assertEqual(r.returncode,0,r.stderr);self.assertEqual(m,['transform_up']);self.assertFalse(p.active)
    def test_lowering(self):
        r,p,m=self.run_client('lower',fsm='loco',extra=['--authorize-motion','--confirm-stationary'])
        self.assertEqual(r.returncode,0,r.stderr);self.assertEqual(m,['transform_down']);self.assertFalse(p.active)
    def test_posture(self):
        r,p,m=self.run_client('pose',fsm='loco',extra=['--authorize-motion','--confirm-stationary','--pitch-deg','5'])
        self.assertEqual(r.returncode,0,r.stderr);self.assertAlmostEqual(p.pose['pitch_rad'],5*3.141592653589793/180);self.assertEqual(m,[])


if __name__=='__main__':unittest.main()
