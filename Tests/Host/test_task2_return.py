#!/usr/bin/env python3
"""Replay return failures and certify full fallback paths including hard parking."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
root=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--exhaustive',action='store_true')
args=parser.parse_args()
with tempfile.TemporaryDirectory(prefix='task2-return-') as directory:
    exe=str(Path(directory)/'test')
    subprocess.run(shlex.split(os.environ.get('CC','gcc'))+[
        '-std=c11','-Wall','-Wextra','-Werror','-pedantic','-O1',
        '-fsanitize=address,undefined','-fno-sanitize-recover=all',
        f'-DTASK2_ROUTE_EXHAUSTIVE_CHECKS={int(args.exhaustive)}','-I'+str(root/'Tests/Inc'),
        '-x','c',str(root/'Tests/Host/task2_return_harness.inc'),str(root/'Tests/Src/Task2Route.c'),'-lm','-o',exe],check=True)
    env=os.environ.copy();env['ASAN_OPTIONS']=env.get('ASAN_OPTIONS','')+':detect_leaks=0'
    subprocess.run([exe],check=True,env=env,timeout=180)
