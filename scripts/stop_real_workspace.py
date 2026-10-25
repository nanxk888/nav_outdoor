"""Stop this checkout's processes, never a machine-wide pkill of ROS/Python."""
import argparse
import os
from pathlib import Path
import signal
import time


def snapshot(proc=Path('/proc')):
    result={}
    for entry in proc.iterdir():
        if not entry.name.isdigit():continue
        try:
            fields=(entry/'stat').read_text().rsplit(')',1)[1].split()
            args=[os.fsdecode(a) for a in (entry/'cmdline').read_bytes().split(b'\0') if a]
            result[int(entry.name)]=(int(fields[1]),fields[19],args)
        except (OSError,ValueError,IndexError):pass
    return result


def targets(table,root):
    scripts={str(root/'scripts'/name) for name in
             ('navigation_web.py','ensure_real_gnss_driver.py','workspace_startup_check.py','start_navigation_when_ready.py')}
    executables={'lio_odometry','real_gnss','real_cloud_filter','lio_memory','navigation_reference','osm_road_planner',
                 'stable_planner','obstacle_memory','gps_goal_to_enu','pose_goal_to_nav','robot_position_marker'}
    selected=set()
    for pid,(_,_,args) in table.items():
        owned_launch=('d1h_nav2' in args and 'real_workspace.launch.py' in args and 'workspace:='+str(root) in args)
        owned_binary=any(a.startswith(str(root)+'/install/') and Path(a).name in executables for a in args)
        owned_bridge=str(root/'map/gnss_bridge.yaml') in args and any(Path(a).name=='domain_bridge' for a in args)
        if owned_launch or owned_binary or owned_bridge or scripts.intersection(args):selected.add(pid)
    # Include children of the selected launch/guard, including their owned GPS/Nav2.
    while True:
        children={pid for pid,(parent,_,_) in table.items() if parent in selected}
        if children<=selected:break
        selected.update(children)
    selected.discard(os.getpid())
    return {pid:table[pid] for pid in selected}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args();root=Path(__file__).resolve().parents[1]
    table=snapshot();owned=targets(table,root)
    if not owned:print('No matching workspace processes.');return
    for pid,(_,_,cmd) in sorted(owned.items()):print('STOP',pid,' '.join(cmd),flush=True)
    if args.dry_run:return
    # Revalidate start time AND argv before every signal to avoid reused PIDs.
    def alive():
        current=snapshot()
        return {pid for pid,old in owned.items() if pid in current and current[pid][1:]==old[1:]}
    for sig,wait in ((signal.SIGINT,8.),(signal.SIGTERM,5.),(signal.SIGKILL,1.)):
        for pid in alive():
            try:os.kill(pid,sig)
            except ProcessLookupError:pass
        deadline=time.monotonic()+wait
        while alive() and time.monotonic()<deadline:time.sleep(.1)
        if not alive():print('Workspace processes stopped. Livox/external drivers were not targeted.');return
    raise SystemExit('Some processes remain: '+str(sorted(alive())))


if __name__=='__main__':main()
