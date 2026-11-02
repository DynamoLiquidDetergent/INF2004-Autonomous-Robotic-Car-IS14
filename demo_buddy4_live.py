"""Print live Buddy 4 serial lines to the terminal. No data files are written."""
from pathlib import Path
from datetime import datetime,timezone,timedelta
import argparse,sys,time

# Optional bundled pyserial when used from the original tools folder.
bundled=Path(__file__).resolve().parent/'phase1-board-test/tools/python'
if bundled.is_dir():sys.path.insert(0,str(bundled))
SG=timezone(timedelta(hours=8))

def format_line(text):
    return '['+datetime.now(SG).isoformat(timespec='milliseconds')+'] '+text.rstrip('\r\n')

def find_port(requested=None):
    from serial.tools import list_ports
    ports=list(list_ports.comports())
    if requested:
        return next((p.device for p in ports if p.device.upper()==requested.upper()),None)
    pico=[p.device for p in ports if p.vid==0x2e8a]
    if len(pico)>1:raise RuntimeError('Multiple Pico ports found. Select one with --port COM4 (use the actual port).')
    return pico[0] if pico else None

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',help='Optional Pico port, for example COM4; otherwise auto-detected')
    parser.add_argument('--plain',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--check','--self-test',action='store_true',dest='check',help='Check formatting only; no serial port or file writes')
    args=parser.parse_args()
    if args.check:
        sample='S4 t_us=20000 state=LEVEL motion=UNCLASSIFIED calibrated=1 valid=1 fresh=1 rc=0 pitch_md=0 roll_md=0 impact=0 hump=0 aborted=0 humps=0 impacts=0 aborts=0 max_us=1000 missed=0'
        assert format_line(sample+'\r\n').endswith('] '+sample)
        print('Formatting check passed using synthetic input. No hardware access or data-file writes.');return
    try:import serial
    except ModuleNotFoundError:
        raise SystemExit('Missing pyserial. Install it using this Python:\n"'+sys.executable+'" -m pip install pyserial')
    if not hasattr(serial,'Serial'):raise SystemExit('Install pyserial; the current serial module does not provide Serial.')
    connection=None;buffer=b'';status=None;last_data=0.0;waiting_message=False
    def notice(message):
        nonlocal status
        if message!=status:print(message,flush=True);status=message
    print('Buddy 4 live sensor output | Ctrl+C to stop | No logs saved',flush=True)
    try:
        while True:
            if connection is None:
                try:
                    port=find_port(args.port)
                    if port is None:
                        notice('Waiting for Pico USB serial...');time.sleep(.5);continue
                    connection=serial.Serial(port,115200,timeout=.25,write_timeout=1,rtscts=False,dsrdtr=False)
                    connection.dtr=True;connection.rts=False;buffer=b'';last_data=time.monotonic();waiting_message=False
                    notice('Connected '+port)
                except (serial.SerialException,OSError) as error:
                    if connection is not None:
                        try:connection.close()
                        except OSError:pass
                    connection=None;notice('Serial unavailable: '+str(error)+' | Close other monitors/check USB; retrying.');time.sleep(1);continue
            try:
                piece=connection.readline()
                if piece:
                    buffer+=piece
                    while b'\n' in buffer:
                        packet,buffer=buffer.split(b'\n',1)
                        text=packet.decode('utf-8',errors='replace').rstrip('\r')
                        if text.startswith('S4 '):
                            print(format_line(text),flush=True);last_data=time.monotonic();waiting_message=False
                    if len(buffer)>65536:
                        buffer=b'';notice('Incomplete serial line discarded; waiting for the next real line.')
                if not waiting_message and time.monotonic()-last_data>5:
                    print('No S4 telemetry received for 5 seconds. Check that the tested Phase 4 firmware is running.',flush=True);waiting_message=True
            except (serial.SerialException,OSError) as error:
                notice('Serial interrupted: '+str(error)+' | Waiting to reconnect.')
                try:connection.close()
                except OSError:pass
                connection=None;buffer=b'';time.sleep(.5)
    except KeyboardInterrupt:
        print('\nStopped. No logs saved.',flush=True)
    finally:
        if connection is not None:
            try:connection.close()
            except OSError:pass
if __name__=='__main__':main()
