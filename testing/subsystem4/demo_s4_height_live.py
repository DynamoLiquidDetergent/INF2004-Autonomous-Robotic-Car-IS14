"""Read-only Buddy 4 live height display. No logs, test grades or motor commands."""
from datetime import datetime,timezone,timedelta
from pathlib import Path
import argparse,math,re,sys,time

REQUIRED={'t_us','state','timebase','calibrated','valid','fresh','rc','bus_rc','init_rc',
          'pitch_md','odom_valid','height_valid','height_mm','highest_valid','highest_mm',
          'hump','humps','height_reason'}
TEXT={'state','motion','timebase','height_reason','profile','build_id','imu_status','imu_policy'}
SG=timezone(timedelta(hours=8))

def parse(line):
    if 'S4 t_us=' not in line:return None
    pairs=re.findall(r'\b([a-z_]+)=([A-Za-z0-9_-]+)',line.split('S4 ',1)[1])
    fields=dict(pairs)
    if len(fields)!=len(pairs) or not REQUIRED.issubset(fields):return None
    try:
        return {key:(value if key in TEXT else int(value)) for key,value in fields.items()}
    except ValueError:return None

def parse_line_status(line):
    # Existing S5 controller trace; do not read/consume S3's RTOS queue here.
    match=re.search(r'\bvc:\s+t=\s*(-?\d+)\s+zone=(\d+)\s+lost=([01])\s+jn=([01])\s+'
                    r'cmd\s+(-?\d+)\s+mm/s\s+steer\s+(-?\d+)\s+wheels\s+'
                    r'L=\s*(-?\d+)\s+R=\s*(-?\d+)\s+dist=\s*(-?\d+)',line)
    if not match:return None
    result=dict(zip(('time_ms','zone','lost','junction','target_mm_s','steer','wheel_l','wheel_r','distance_mm'),map(int,match.groups())))
    return result if 0<=result['zone']<=7 else None

def render_line(info):
    return (f'LINE | Zone={info["zone"]} | Lost={"YES" if info["lost"] else "NO"} | '
            f'Junction={"YES" if info["junction"] else "NO"} | Target={info["target_mm_s"]} mm/s | '
            f'Wheels L/R={info["wheel_l"]}/{info["wheel_r"]} mm/s')

def healthy(row):
    return (row['timebase']=='CONSISTENT' and row['state']!='FAULT' and row['calibrated']==1
            and row['valid']==1 and row['fresh']==1 and row['rc']==0 and row['bus_rc']==0
            and row['init_rc']==0 and row.get('fault_rc',0)==0)

def height(row):
    return str(row['height_mm'])+' mm' if healthy(row) and row['odom_valid']==1 and row['height_valid']==1 and row['height_mm']>=0 else '--'

def highest(row):
    return str(row['highest_mm'])+' mm' if row['highest_valid']==1 and row['highest_mm']>=0 else '--'

def restarted(previous,row):
    if previous is None:return False
    return (((row['t_us']-previous['t_us'])&0xffffffff)>=0x80000000 or
            ((row['humps']-previous['humps'])&0xffffffff)>=0x80000000)

def offset_values(row,offset):
    """Display-only experiment. Never changes firmware values or validity."""
    raw=row['height_mm'] if height(row)!='--' else None
    corrected=None if raw is None else raw-offset
    return raw,corrected

def offset_summary(row,offset):
    raw,corrected=offset_values(row,offset)
    raw_text='--' if raw is None else f'{raw:g} mm'
    corrected_text='--' if corrected is None else (f'{corrected:g} mm' if corrected>=0 else f'INVALID ({corrected:g} mm below zero)')
    best=row['highest_mm']-offset if row['highest_valid']==1 and row['highest_mm']>=0 else None
    best_text='--' if best is None else (f'{best:g} mm' if best>=0 else f'INVALID ({best:g} mm below zero)')
    return (f'Raw height={raw_text} | Corrected height={corrected_text} | Subtracted offset={offset:g} mm'
            f' | Raw highest={highest(row)} | Corrected highest={best_text} | OFFSET EXPERIMENT - UNVALIDATED')

def render(row,offset=None):
    clock={'CONSISTENT':'READY','PENDING':'WAITING','BAD':'FAULT'}.get(row['timebase'],'UNKNOWN')
    pitch=f'{row["pitch_md"]/1000:+.1f} deg' if healthy(row) else '--'
    return (f'State={row["state"]:<12} | Pitch={pitch:<10} | Clock={clock:<7} | '
            f'Cal={"YES" if row["calibrated"]==1 else "NO":<3} | '
            f'Encoder={"LIVE" if row.get("encoder_valid",row["odom_valid"])==1 else "UNAVAILABLE"} | '
            +(f'Last height={height(row)} | Highest stored={highest(row)}' if offset is None else offset_summary(row,offset))
            +f' | Humps={row["humps"]}'
            +(f' | IMU={row["imu_status"]} | Interpolated={row.get("interpolated",0)}' if 'imu_status' in row else '')
            +(f' | Accel norm error={row["dynamic_mg"]/1000:.3f} g | Pending IMU={row.get("pending_imu",0)}' if 'dynamic_mg' in row else '')
            +(f' | Height blocked={row["height_reason"]}' if row.get('chain_valid',1)==0 and row['height_reason']!='OK' else ''))

def event(row,reference=None,captured=True,offset=None):
    prefix=f'HUMP #{row["humps"]}' if captured else f'LAST REPORTED HUMP #{row["humps"]} (completion event not captured)'
    if height(row)=='--':return prefix+' | HEIGHT UNAVAILABLE | Reason='+row['height_reason']
    result=prefix+' | '+('Estimated height='+height(row)+' | Highest stored='+highest(row) if offset is None else offset_summary(row,offset))
    if row.get('interpolated',0):
        label='Skipped IMU samples' if row.get('right_wheel_mode')==1 else 'Interpolated samples'
        result+=f' | {label}={row["interpolated"]} | Longest interval={row.get("max_interp_us",0)/1000:g} ms'
    if reference is not None:
        error=abs(row['height_mm']-reference)/reference*100
        if offset is None:result+=f' | Reference={reference:g} mm | Error={error:.1f}%'
        else:
            corrected=row['height_mm']-offset
            error_text=f'{abs(corrected-reference)/reference*100:.1f}%' if corrected>=0 else 'N/A (negative corrected height)'
            result+=f' | Reference={reference:g} mm | Raw error={error:.1f}% | Corrected error={error_text}'
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',help='Optional Pico port, for example COM4')
    parser.add_argument('--reference-mm',type=float,help='Optional independently measured height in mm; no PASS/FAIL is assigned')
    parser.add_argument('--raw',action='store_true',help='Also print original S4 serial lines for diagnosing a rejected height; no files are saved')
    parser.add_argument('--duration-seconds',type=float,help='End the display after this many seconds; does NOT stop the motors')
    parser.add_argument('--height-offset-mm',type=float,default=None,help='Display-only test: corrected = raw minus this offset in mm; does not change firmware')
    args=parser.parse_args()
    if args.reference_mm is not None and (not math.isfinite(args.reference_mm) or args.reference_mm<=0):
        parser.error('--reference-mm must be a positive measured value')
    if args.duration_seconds is not None and (not math.isfinite(args.duration_seconds) or args.duration_seconds<=0):
        parser.error('--duration-seconds must be positive')
    if args.height_offset_mm is not None and (not math.isfinite(args.height_offset_mm) or abs(args.height_offset_mm)>1000):
        parser.error('--height-offset-mm must be finite and between -1000 and 1000')
    bundled=Path(__file__).resolve().parent/'phase1-board-test/tools/python'
    if bundled.is_dir():sys.path.insert(0,str(bundled))
    import serial
    from serial.tools import list_ports
    print('BUDDY 4 | LIVE HUMP HEIGHT | Ctrl+C closes display | No files saved',flush=True)
    print('This monitor only reads sensor output. Start/stop behaviour belongs to the flashed firmware.',flush=True)
    print('Ctrl+C does not stop the motors. Use the car power switch if a physical stop is needed.',flush=True)
    if args.reference_mm is not None:print(f'Measured reference: {args.reference_mm:g} mm ({args.reference_mm/10:g} cm). Used for comparison only.',flush=True)
    if args.height_offset_mm is not None:print(f'UNVALIDATED OFFSET EXPERIMENT: corrected = raw minus {args.height_offset_mm:g} mm. Firmware values, counters and validity are unchanged.',flush=True)
    connection=None;previous=None;shown_event=None;last_received=time.monotonic();stale=False;profile=None;pending=b''
    last_line=None;line_stale=False
    display_started=time.monotonic()
    try:
        print('Waiting for Pico USB serial...',flush=True)
        while True:
            if args.duration_seconds is not None and time.monotonic()-display_started>=args.duration_seconds:
                print('Display timer ended. This does NOT stop the motors; use the physical switch. No files written by the monitor.',flush=True)
                break
            if connection is None:
                ports=list(list_ports.comports())
                matches=[p.device for p in ports if p.device.upper()==args.port.upper()] if args.port else [p.device for p in ports if p.vid==0x2e8a]
                if len(matches)>1:raise RuntimeError('More than one Pico found. Disconnect the other board or select --port COMx.')
                if not matches:time.sleep(.3);continue
                try:
                    connection=serial.Serial(matches[0],115200,timeout=.25,rtscts=False,dsrdtr=False)
                    connection.dtr=True;connection.rts=False
                    last_received=time.monotonic();stale=False;pending=b''
                    print('Connected '+connection.port,flush=True)
                except (serial.SerialException,OSError):
                    if connection:connection.close()
                    connection=None;time.sleep(.5);continue
            try:
                piece=connection.readline()
            except (serial.SerialException,OSError):
                connection.close();connection=None;pending=b''
                print('USB disconnected. Waiting for Pico...',flush=True);continue
            pending+=piece
            if len(pending)>65536:raise RuntimeError('Serial line exceeded 64 KiB; check the flashed firmware.')
            while b'\n' in pending:
                packet,pending=pending.split(b'\n',1)
                text=packet.decode('utf-8',errors='replace').rstrip()
                stamp=datetime.now(SG).strftime('%H:%M:%S')
                is_s4='S4 t_us=' in text
                # Preserve evidence BEFORE parsing: a new/unrecognised field
                # must not silently erase the original firmware output.
                if args.raw and (text.startswith('S2,') or is_s4):
                    print(f'[{stamp}] RAW '+text,flush=True)
                info=parse_line_status(text)
                if info:
                    print(f'[{stamp}] '+render_line(info),flush=True)
                    last_line=time.monotonic();line_stale=False
                elif text.startswith(('S4 HEIGHT ONLY:','S4_HEIGHT_ONLY ', 'vc: calibrating','vc: calibration','vc: put the robot','vc: following',
                                      'vc: ready','vc: mission state','vc: hump traversal','safety:')):
                    print(f'[{stamp}] '+text,flush=True)
                row=parse(text)
                if row:
                    now=datetime.now(SG).strftime('%H:%M:%S')
                    if restarted(previous,row):
                        print(f'[{now}] NEW BOOT: counters/highest restart for this run.',flush=True);shown_event=None;previous=None;last_line=None;line_stale=False
                    mode=row.get('profile','UNKNOWN')
                    if mode!=profile:
                        label={'HEIGHT_ONLY':'powered straight height demo; starts after healthy startup',
                               'CLOCK_DIAG':'stationary clock diagnostic; drive task disabled',
                               'LINE_HEIGHT':'line following + live hump height',
                               'PUSH':'hand-pushed integration image'}.get(mode,mode+' firmware')
                        print(f'[{now}] Mode: {label} | Firmware={row.get("build_id","UNKNOWN")}',flush=True);profile=mode
                    print(f'[{now}] '+render(row,args.height_offset_mm),flush=True)
                    if row.get('right_wheel_mode')==1:
                        print(f'[{now}] RIGHT-WHEEL MODE | Roll={row["roll_md"]/1000:+.2f} deg | Track={row["track_um"]/1000:g} mm | Left tyre must stay on flat floor',flush=True)
                    if row.get('filter_compare')==1:
                        names={0:'INITIALISING',1:'CALIBRATING',2:'LEVEL',3:'CLIMBING',4:'DESCENDING',5:'FAULT'}
                        print(f'[{now}] FILTER | Today pitch/roll={row["pitch_md"]/1000:+.2f}/{row["roll_md"]/1000:+.2f} deg'
                              f' valid={row["valid"]} state={row["state"]}'
                              f' | Yesterday pitch/roll={row["old_pitch_md"]/1000:+.2f}/{row["old_roll_md"]/1000:+.2f} deg'
                              f' valid={row["old_valid"]} state={names.get(row["old_state"],"UNKNOWN")}'
                              f' | Rejected samples={row["diag_rejected"]} longest run={row["diag_longest"]}'
                              ' | Yesterday is observation only'
                              +(' (reference state uses the old pitch model; compare angles only)' if row.get('right_wheel_mode')==1 else ''),flush=True)
                    changed=previous is not None and 0<((row['humps']-previous['humps'])&0xffffffff)<0x80000000
                    reported=previous is None and row['humps']>0 and row['height_valid']==1
                    if (row['hump']==1 or changed or reported) and row['humps']!=shown_event:
                        print(f'[{now}] >>> '+event(row,args.reference_mm,row['hump']==1,args.height_offset_mm),flush=True)
                        shown_event=row['humps']
                    previous=row;last_received=time.monotonic();stale=False
                elif is_s4:
                    print(f'[{stamp}] S4 row could not be parsed; no reading claimed.'+
                          (' Original row retained above.' if args.raw else ' Run with --raw to preserve the original row.'),flush=True)
            if not stale and time.monotonic()-last_received>5:
                print('No current S4 telemetry for 5 seconds. Waiting; no new height reading is available.',flush=True);stale=True
            if profile=='LINE_HEIGHT' and last_line is not None and not line_stale and time.monotonic()-last_line>3:
                print('No fresh line/controller report for 3 seconds. The last line status is stale.',flush=True);line_stale=True
    except KeyboardInterrupt:
        print('\nMonitor stopped. No files or test records written.')
    finally:
        if connection:connection.close()

if __name__=='__main__':
    try:main()
    except Exception as error:print('Stopped:',error,file=sys.stderr);raise SystemExit(1)
