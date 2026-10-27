#!/usr/bin/env python3
"""Portable C tests. Hardware and RTOS scheduling need separate board tests."""
from pathlib import Path
import argparse,subprocess,sys,os
p=argparse.ArgumentParser()
p.add_argument('--compiler',default='cc')
p.add_argument('--zig',action='store_true')
p.add_argument('--build-dir',default=None)
args=p.parse_args()
root=Path(__file__).resolve().parents[2]
build=Path(args.build_dir).resolve() if args.build_dir else root.parent/'s4-host-build'
build.mkdir(parents=True,exist_ok=True)
os.environ['ZIG_GLOBAL_CACHE_DIR']=str(build/'zig-global')
os.environ['ZIG_LOCAL_CACHE_DIR']=str(build/'zig-local')
os.environ['TEMP']=str(build)
os.environ['TMP']=str(build)
source=root/'app_program/subsystem4'
portable=['s4_time.c','drv_motion_sensor.c','s4_math.c','s4_processing.c','s4_terrain.c','s4_odometry.c','s4_packing.c']
portable=[str(source/x) for x in portable if (source/x).exists()]
for test in sorted((Path(__file__).parent/'host').glob('test_*.c')):
    if test.stem=='test_transport':
        extra=[str(source/'s4_transport.c')]
        flags=['-DS4_HOST_TEST','-I'+str(Path(__file__).parent/'stubs')]
    else:
        extra=[];flags=[]
    exe=build/(test.stem+('.exe' if sys.platform=='win32' else ''))
    cmd=[args.compiler]+(['cc'] if args.zig else [])+['-std=c11','-UNDEBUG','-Wall','-Wextra','-Werror','-O1','-I'+str(source)]+flags+[str(test)]+portable+extra+['-lm','-o',str(exe)]
    subprocess.run(cmd,check=True)
    subprocess.run([str(exe)],check=True)
print('All available host tests passed. Board tests: NOT RUN.')
