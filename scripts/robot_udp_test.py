"""Orin commissioning client. Default status only; no ROS or SSH control."""
import argparse
import secrets
import socket
import time
from pathlib import Path
from robot_wire import encode,decode
from robot_policy import posture


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--bind',default='192.168.19.91');p.add_argument('--robot',default='192.168.19.90')
    p.add_argument('--port',type=int,default=10099);p.add_argument('--key-file',required=True)
    p.add_argument('--action',choices=['status','stand','loco','lower','pose','hold','stop'],default='status')
    p.add_argument('--authorize-motion',action='store_true');p.add_argument('--confirm-stationary',action='store_true')
    p.add_argument('--roll-deg',type=float,default=0.);p.add_argument('--pitch-deg',type=float,default=0.)
    p.add_argument('--height-m',type=float);p.add_argument('--duration',type=float,default=10.)
    a=p.parse_args()
    if a.action not in ('status','stop') and not a.authorize_motion:p.error('Explicit --authorize-motion required')
    if a.action in ('loco','lower','pose') and not a.confirm_stationary:p.error('Confirm actual robot is stationary with --confirm-stationary')
    if not 1<=a.duration<=60:p.error('Duration must be 1..60 seconds')
    import math
    pose={'roll_rad':math.radians(a.roll_deg),'pitch_rad':math.radians(a.pitch_deg)}
    if a.height_m is not None:pose['height_m']=a.height_m
    if a.action=='pose':posture(pose,allow_height=True)
    key=Path(a.key_file).read_bytes().strip()
    if len(key)<32:p.error('Shared key must contain at least 32 bytes')
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind((a.bind,0));sock.settimeout(.12)
    target=(a.robot,a.port);seq=0;session=secrets.token_hex(16);operation=secrets.token_hex(8);state={}
    def exchange(kind,**fields):
        nonlocal seq,state
        seq+=1
        data=dict(protocol=2,time=time.time(),kind=kind,seq=seq,session=session,
                  nonce=state.get('nonce',''),operation=operation,v=0.,w=0.,**fields)
        sock.sendto(encode(data,key),target)
        end=time.monotonic()+.15
        while time.monotonic()<end:
            raw,addr=sock.recvfrom(4097)
            if addr!=target:continue
            try:r=decode(raw,key)
            except (ValueError,KeyError,TypeError):continue
            if r.get('protocol')==2 and r.get('kind')=='status' and r.get('reply_to')==seq:
                state=r
                if r.get('command_accepted') is False:raise RuntimeError('Receiver rejected: '+r.get('rejection',''))
                return r
        raise TimeoutError('No fresh authenticated reply')
    armed=False
    try:
        exchange('hello');print(state,flush=True)
        if a.action=='status':return
        if a.action=='stop':
            if state.get('active'):
                session=state['session'];seq=max(seq,state['ack']);exchange('stop')
            return
        if state.get('active'):raise RuntimeError('Another session is active; stop its owner first')
        if not all(state.get(k) for k in ('authorized','sdk_enabled','biped','fsm_fresh','interfaces_fresh')):
            raise RuntimeError('Receiver authorization / SDK mode / biped / fresh feedback not ready')
        if a.action=='stand' and state.get('fsm')!='idle':raise RuntimeError('stand requires idle')
        exchange('arm');armed=state.get('active') and state.get('session')==session
        if not armed:raise RuntimeError('Receiver did not acknowledge arm')
        start=time.monotonic();last_report=0.
        while time.monotonic()-start<a.duration:
            fields={}
            if time.monotonic()-start>.65:
                if a.action in ('stand','loco','lower'):
                    fields={'mode':{'stand':'transform_up','loco':'loco','lower':'transform_down'}[a.action],
                            'confirm_stationary':a.confirm_stationary}
                elif a.action=='pose':fields={'pose':pose}
            exchange('drive',**fields)
            if state.get('fault') or not state.get('active'):raise RuntimeError('Receiver safety stop: '+state.get('reason',''))
            if time.monotonic()-last_report>1:
                print('FSM:',state.get('fsm'),'operation:',state.get('operation'),'pose:',state.get('pose'),flush=True);last_report=time.monotonic()
            time.sleep(.035)
        print('Test finished; sending zero-speed session stop. Verify actual stance/stop on site.')
    finally:
        if armed:
            try:exchange('stop')
            except Exception:print('Stop acknowledgement unavailable; receiver watchdog must stop velocity. Check hardware.')
        sock.close()


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:pass
    except Exception as exc:raise SystemExit('Robot test failed: '+str(exc))
