"""Humble launch integration: one failure must not terminate the other child."""
import importlib.util
from pathlib import Path
import signal
import sys
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnProcessStart

path=Path(__file__).resolve().parents[1]/'src/d1h_nav2/launch/real_workspace.launch.py'
spec=importlib.util.spec_from_file_location('workspace_launch',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
failed=ExecuteProcess(cmd=[sys.executable,'-c','import time;time.sleep(.5);raise SystemExit(7)'])
survivor=ExecuteProcess(cmd=[sys.executable,'-c','import time;time.sleep(30)'])
seen=[]
service=LaunchService()
def exited(event,context):
    seen.append(event.returncode)
    return module.report_process_exit(event,context)
def interrupt(context):
    # Simulate terminal Ctrl+C through LaunchService's signal handler.
    import os
    os.kill(os.getpid(),signal.SIGINT)
    return []
description=LaunchDescription([
    RegisterEventHandler(OnProcessExit(target_action=failed,on_exit=exited)),
    RegisterEventHandler(OnProcessStart(target_action=survivor,on_start=lambda e,c:seen.append('started'))),
    RegisterEventHandler(OnProcessExit(target_action=survivor,on_exit=lambda e,c:seen.append('stopped'))),
    failed,survivor,TimerAction(period=2.,actions=[OpaqueFunction(function=interrupt)])])
service.include_launch_description(description)
service.run()
assert seen==['started',7,'stopped'],seen
print('PASS: child failure isolated; Ctrl+C stops survivor')
