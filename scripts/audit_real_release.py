"""Audit exact staged publication scope; does not print possible secret values."""
import ast
import json
from pathlib import Path
import re
import subprocess
import sys

root=Path(__file__).resolve().parents[1]
def git(*args):return subprocess.check_output(['git',*args],cwd=root)


def main():
    names=git('ls-files','-z').decode().split('\0')[:-1]
    failures=[];total=0
    packages={'d1h_fast_lio','d1h_global_localization','d1h_nav2'}
    secret=re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----|\bgh[pousr]_[A-Za-z0-9]{30,}|\bgithub_pat_[A-Za-z0-9_]{30,}|\bAKIA[A-Z0-9]{16}\b')
    for name in names:
        path=Path(name)
        if (any(x in {'build','install','log','__pycache__','.local','third_party'} for x in path.parts)
            or path.suffix.lower() in {'.pyc','.bag','.mcap','.db3','.pcd','.ply','.onnx','.pt','.token','.pem','.key'}
            or (path.parts[0]=='src' and path.parts[1] not in packages)):
            failures.append((name,'forbidden publication scope'))
        data=git('show',':'+name);total+=len(data)
        if len(data)>10*1024*1024:failures.append((name,'file exceeds 10MiB release budget'))
        if secret.search(data):failures.append((name,'possible credential; value withheld'))
        if path.suffix=='.py':
            try:ast.parse(data.decode('utf-8-sig'),filename=name)
            except (SyntaxError,UnicodeError) as exc:failures.append((name,str(exc)))
    # Explicit setup.py data files must exist in a fresh checkout, not just locally.
    for package in ('d1h_global_localization',):
        tree=ast.parse(git('show',f':src/{package}/setup.py').decode())
        for node in ast.walk(tree):
            if isinstance(node,ast.Constant) and isinstance(node.value,str) and node.value.startswith(('launch/','config/')):
                target=f'src/{package}/{node.value}'
                if target not in names:failures.append((target,'setup.py references an unpublished file'))
    print(json.dumps(dict(files=len(names),total_bytes=total,failures=failures),indent=2))
    return bool(failures)


if __name__=='__main__':sys.exit(main())
