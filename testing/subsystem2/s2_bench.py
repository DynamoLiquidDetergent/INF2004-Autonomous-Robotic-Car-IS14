#!/usr/bin/env python3
"""
s2_bench.py - guided bench tests for Subsystem 2 (Motion Control).

Tests the Buddy 2 parts on their own: Pico W + Robo Pico + 2 motors +
2 encoders.  No chassis, no other subsystems' hardware needed.

What it does for each test:
  1. tells you what to connect and what to do, step by step
  2. sets the debug switches (S2_TEST_DRIVE, S2_LOG_EVERY_CYCLES)
  3. builds the test firmware through WSL and copies it to the Pico
  4. reads the USB serial output, decides PASS / FAIL, saves the log
  5. puts the debug switches back to 0 when you quit

Run it in PowerShell from the repository root (not inside WSL, because
WSL cannot see the Pico's COM port):

    py -m pip install pyserial          (once)
    py testing\\subsystem2\\s2_bench.py

Options:
    --port COM5      use this serial port instead of finding the Pico
    --no-build       do not build or flash; you flash the .uf2 yourself

Calibration values you accept (invert flags, pulses per turn, gains) are
written into the source files and kept.  Debug switches are always reset.
"""

import argparse
import datetime
import math
import os
import queue
import re
import shutil
import string
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pyserial is missing.  Run:  py -m pip install pyserial")
    sys.exit(1)

# ---------------------------------------------------------------- paths
REPO = Path(__file__).resolve().parents[2]
S2_DIR = REPO / "app_program" / "subsystem2"
BUILD_DIR = REPO / "build_make"
LOG_DIR = REPO / "testing" / "subsystem2" / "logs"
TEST_LOG = REPO / "testing" / "subsystem2" / "TEST_LOG.md"
UF2 = BUILD_DIR / "mtk3pico_smp1_usb_cdc_wifi_test.uf2"

F_TEST = S2_DIR / "subsystem2_test.c"
F_CTRL = S2_DIR / "motion_control.c"
F_HDR = S2_DIR / "subsystem2.h"
F_MATH = S2_DIR / "motion_math.h"

DEBUG_SWITCHES = {"S2_TEST_DRIVE": F_TEST, "S2_TEST_TURN": F_TEST, "S2_LOG_EVERY_CYCLES": F_CTRL}

PICO_VID = 0x2E8A
BAUD = 115200
IS_WIN = os.name == "nt"

# state numbers from subsystem2.h
STATE = {0: "IDLE", 1: "DRIVING", 2: "TURNING", 3: "DRIVING_DIST",
         4: "HOLDING", 5: "INHIBITED", 6: "STALLED", 7: "TIMEOUT"}
UNIT_IDS = ["S2-U%02d" % i for i in range(1, 12)]

# ---------------------------------------------------------------- output
if IS_WIN:
    os.system("")  # turn on ANSI colours in the Windows console

def c(code, s):
    return "\033[%sm%s\033[0m" % (code, s)

def title(s):
    print()
    print(c("1;36", "=" * 64))
    print(c("1;36", "  " + s))
    print(c("1;36", "=" * 64))

def step(n, s):
    print(c("1", "\n  [%s] " % n) + s)

def say(s=""):
    print("      " + s)

def good(s):
    print(c("1;32", "  PASS  ") + s)

def bad(s):
    print(c("1;31", "  FAIL  ") + s)

def warn(s):
    print(c("1;33", "  NOTE  ") + s)

def ask(s, default=""):
    try:
        r = input(c("1;35", "  > ") + s + (" [%s] " % default if default else " "))
    except EOFError:
        r = ""
    return r.strip() or default

def ask_num(s, default="", allow_blank=False):
    while True:
        r = ask(s, default)
        if allow_blank and r == "":
            return None
        try:
            v = float(r)
            if v > 0:
                return v
        except ValueError:
            pass
        say("Please type a number (e.g. 65).")

def yes(s, default="y"):
    return ask(s + " (y/n)", default).lower().startswith("y")

def enter(s="Press Enter when done"):
    ask(s + " ...")

def danger_wheels_up():
    print()
    print(c("1;41;97", "  WHEELS UP: the motors spin as soon as the Pico boots.        "))
    print(c("1;41;97", "  Wheels must hang free (robot on a box, or motors loose).     "))

# ---------------------------------------------------------------- source edits
def read_define(path, name):
    m = re.search(rb"#define[ \t]+" + name.encode() + rb"[ \t]+(\S+)", path.read_bytes())
    return m.group(1).decode() if m else None

def write_define(path, name, value):
    """Replace the value of '#define NAME value', keeping CRLF and comments."""
    data = path.read_bytes()
    pat = re.compile(rb"(#define[ \t]+" + name.encode() + rb"[ \t]+)(\S+)")
    new, n = pat.subn(lambda m: m.group(1) + str(value).encode(), data, count=1)
    if n != 1:
        raise RuntimeError("could not find #define %s in %s" % (name, path.name))
    if new != data:
        path.write_bytes(new)

ORIGINAL = {}

def set_switch(name, value):
    path = DEBUG_SWITCHES[name]
    if name not in ORIGINAL:
        ORIGINAL[name] = read_define(path, name)
    write_define(path, name, value)

def restore_switches():
    for name, val in ORIGINAL.items():
        if val is not None:
            write_define(DEBUG_SWITCHES[name], name, val)
    if ORIGINAL:
        print(c("2", "\n  Debug switches put back: " +
                ", ".join("%s %s" % kv for kv in ORIGINAL.items())))

def calibrate(path, name, value, why):
    old = read_define(path, name)
    if old == str(value):
        say("%s is already %s." % (name, value))
        return
    if yes("Set %s from %s to %s in %s? (%s)" % (name, old, value, path.name, why)):
        write_define(path, name, value)
        CHANGES.append("%s %s -> %s" % (name, old, value))
        good("%s = %s saved" % (name, value))

CHANGES = []

# ---------------------------------------------------------------- build + flash
def build():
    step("B", "Building the test firmware (make TEST=1) ...")
    # Remove subsystem 2's objects so they are always recompiled with the
    # current debug switches.  Without this, make can miss the change when
    # the WSL clock runs ahead of Windows ("Nothing to be done for 'all'").
    obj_dir = BUILD_DIR / "mtkernel_3" / "app_program" / "subsystem2"
    for f in list(obj_dir.glob("*.o")) + list(obj_dir.glob("*.d")):
        try:
            f.unlink()
        except OSError:
            pass
    before = UF2.stat().st_mtime if UF2.exists() else None
    if IS_WIN:
        cmd = ["wsl", "bash", "-ic", "make TEST=1 -j8"]
    else:
        cmd = ["bash", "-ic", "make TEST=1 -j8"]
    try:
        r = subprocess.run(cmd, cwd=str(BUILD_DIR))
    except FileNotFoundError:
        bad("Could not start WSL.  Build by hand in WSL:  cd build_make && make TEST=1 -j8")
        enter("Press Enter after building by hand")
        return UF2.exists()
    ok = (r.returncode == 0 and UF2.exists()
          and (before is None or UF2.stat().st_mtime != before))
    if ok:
        good("built %s" % UF2.name)
    elif r.returncode == 0 and UF2.exists():
        bad("make finished but %s was not rebuilt." % UF2.name)
        say("In WSL run:  cd build_make && make clean && make TEST=1 -j8  then try again.")
    else:
        bad("build failed (scroll up for the first 'error:').")
        say("If it says PICO_SDK_PATH or arm-none-eabi-gcc: finish README section 1.")
        say("('cannot set terminal process group' / 'no job control' are harmless.)")
    return ok

def find_bootsel_drive():
    if not IS_WIN:
        return None
    for letter in string.ascii_uppercase[3:]:
        root = Path("%s:/" % letter)
        try:
            if (root / "INFO_UF2.TXT").exists():
                return root
        except OSError:
            pass
    return None

def flash():
    step("F", "Put the Pico in BOOTSEL mode:")
    say("1. Unplug the USB cable (switch motor battery OFF if it also powers the Pico).")
    say("2. Hold the BOOTSEL button, plug USB back in, then let go.")
    say("   A drive called RPI-RP2 appears.")
    if not IS_WIN:
        enter("Copy %s onto RPI-RP2 yourself, then press Enter" % UF2)
        return True
    t0 = time.time()
    drive = None
    while drive is None:
        drive = find_bootsel_drive()
        if drive is None:
            if time.time() - t0 > 120:
                bad("no RPI-RP2 drive after 2 minutes.")
                return False
            time.sleep(0.5)
    say("Found RPI-RP2 at %s - copying %s ..." % (drive, UF2.name))
    try:
        shutil.copyfile(str(UF2), str(drive / UF2.name))
    except OSError:
        pass  # the drive vanishes as soon as the Pico reboots; that is normal
    good("firmware copied; the Pico is rebooting")
    say("Switch the motor battery ON now if the test needs the motors.")
    return True

# ---------------------------------------------------------------- serial reader
def pico_ports():
    ports = list(list_ports.comports())
    return [p.device for p in ports if p.vid == PICO_VID] or None

class Reader(threading.Thread):
    """Reads the Pico's serial port in the background, reconnecting when the
    Pico reboots or the cable is replugged.  Saves every line to a log."""

    def __init__(self, fixed_port, logpath):
        super().__init__(daemon=True)
        self.fixed_port = fixed_port
        self.log = open(logpath, "w", encoding="utf-8", newline="\n")
        self.logpath = logpath
        self.lines = queue.Queue()
        self.rows = []          # parsed S2 CSV rows (dicts)
        self.last_row = None
        self.connected = False
        self.stop_flag = False

    def port_name(self):
        if self.fixed_port:
            return self.fixed_port
        p = pico_ports()
        return p[0] if p else None

    def run(self):
        buf = b""
        while not self.stop_flag:
            name = self.port_name()
            if not name:
                time.sleep(0.3)
                continue
            try:
                with serial.Serial(name, BAUD, timeout=0.2) as s:
                    self.connected = True
                    while not self.stop_flag:
                        data = s.read(512)
                        if not data:
                            continue
                        buf += data
                        while b"\n" in buf:
                            raw, buf = buf.split(b"\n", 1)
                            self._line(raw.decode("utf-8", "replace").rstrip("\r"))
            except (serial.SerialException, OSError):
                pass
            self.connected = False
            time.sleep(0.3)

    def _line(self, line):
        stamp = time.time()
        try:
            self.log.write(line + "\n")
            self.log.flush()
        except ValueError:
            return  # log already closed
        if line.startswith("S2,"):
            f = line.split(",")
            if len(f) == 10:
                try:
                    v = [int(x) for x in f[1:]]
                    row = dict(t=v[0], state=v[1], sp_l=v[2], spd_l=v[3], duty_l=v[4],
                               sp_r=v[5], spd_r=v[6], duty_r=v[7], dist=v[8], pc=stamp)
                    self.rows.append(row)
                    self.last_row = row
                except ValueError:
                    pass
            return
        self.lines.put(line)

    def close(self):
        self.stop_flag = True
        time.sleep(0.3)
        self.log.close()


def show(line):
    if "[TEST]" in line:
        if " PASS" in line:
            print("    " + c("32", line))
        elif " FAIL" in line:
            print("    " + c("1;31", line))
        else:
            print("    " + c("33", line))
    else:
        print(c("2", "    " + line))

def wait_for(reader, pattern, timeout, quiet=False, hint_after=12):
    """Print serial lines until one matches pattern. Returns the match or None."""
    rx = re.compile(pattern)
    t0 = time.time()
    seen_any = False
    hinted = False
    while time.time() - t0 < timeout:
        try:
            line = reader.lines.get(timeout=0.25)
        except queue.Empty:
            if not seen_any and not hinted and time.time() - t0 > hint_after:
                hinted = True
                if not reader.connected:
                    warn("No Pico serial port found yet.")
                    say("Ports seen: " + (", ".join(p.device for p in list_ports.comports()) or "none"))
                    say("If the Pico is one of them, restart with  --port COMx")
                else:
                    warn("Port open but no output: the start was probably missed.")
                say("Unplug USB and plug it back in (no BOOTSEL). The script reconnects.")
                say("If the motor battery also powers the Pico, switch it off/on too.")
            continue
        seen_any = True
        if not quiet:
            show(line)
        m = rx.search(line)
        if m:
            return m
    return None

def open_reader(args, tag):
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    name = "%s_%s.txt" % (datetime.datetime.now().strftime("%Y-%m-%d_%H%M"), tag)
    r = Reader(args.port, LOG_DIR / name)
    r.start()
    return r

def prepare(args, switches):
    """Set the debug switches, build, flash.  Returns False to abort."""
    for k, v in switches.items():
        set_switch(k, v)
    say("Settings for this test: " + ", ".join("%s %s" % kv for kv in switches.items()))
    if args.no_build:
        enter("Build with  make TEST=1 -j8  in WSL, flash the .uf2, then press Enter")
        return True
    if not build():
        return False
    return flash()

RESULTS = []   # (test_id, PASS/FAIL/SKIP, evidence, note)

def result(tid, ok, evidence, note=""):
    res = ok if isinstance(ok, str) else ("PASS" if ok else "FAIL")
    RESULTS.append((tid, res, evidence, note))
    (good if res == "PASS" else bad if res == "FAIL" else warn)("%s %s" % (tid, note))

def rel(p):
    return Path(p).relative_to(REPO / "testing" / "subsystem2").as_posix()

# ================================================================ tests
def t_host(args):
    title("PC maths tests  S2-H01..H12  (no Pico needed)")
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    out = LOG_DIR / ("%s_host.txt" % datetime.date.today().isoformat())
    # Same steps as host/run_host_tests.sh, run directly: a Windows checkout
    # may have given the .sh file CRLF line endings, which sh rejects.
    sh_cmd = ("cc -std=c99 -Wall -Wextra -DS2_HOST_TEST -Itesting/subsystem2/host "
              "testing/subsystem2/host/test_motion_math.c app_program/subsystem2/motion_math.c "
              "-o /tmp/s2_host_tests && /tmp/s2_host_tests")
    cmd = ["wsl", "sh", "-c", sh_cmd] if IS_WIN else ["sh", "-c", sh_cmd]
    try:
        p = subprocess.run(cmd, cwd=str(REPO), capture_output=True, text=True)
    except FileNotFoundError:
        bad("could not start WSL.")
        return
    text = p.stdout + p.stderr
    out.write_text(text, encoding="utf-8")
    for line in text.splitlines():
        show(line)
    m = re.search(r"SUMMARY (\d+) passed, (\d+) failed", text)
    if not m:
        bad("no SUMMARY line. Is gcc installed in WSL?  sudo apt install build-essential")
        return
    for tid in ["S2-H%02d" % i for i in range(1, 13)]:
        hit = re.search(re.escape(tid) + r" (PASS|FAIL)", text)
        if hit:
            RESULTS.append((tid, hit.group(1), rel(out), ""))
    (good if m.group(2) == "0" else bad)("%s passed, %s failed  -> %s" % (m.group(1), m.group(2), rel(out)))


def t_unit(args):
    title("Automatic tests on the Pico  S2-U01..U11  (motors stay off)")
    step(1, "Connect: Pico W on the Robo Pico, USB to the laptop.")
    say("Motors and encoders may be connected or not - nothing moves.")
    enter()
    if not prepare(args, {"S2_TEST_DRIVE": 0, "S2_LOG_EVERY_CYCLES": 0}):
        return
    r = open_reader(args, "unit")
    step(2, "Reading the Pico's output (other subsystems print too) ...")
    m = wait_for(r, r"\[TEST\] subsystem2 SUMMARY (\d+) passed, (\d+) failed", 90)
    r.close()
    text = r.logpath.read_text(encoding="utf-8")
    if not m:
        bad("no 'subsystem2 SUMMARY' line within 90 s.  Log: %s" % rel(r.logpath))
        say("If the output stopped at another subsystem's test, it may be waiting for")
        say("its hardware: comment out the other subsystemN_test() calls in app_main.c")
        say("(locally only, do not commit) and run this again.")
        return
    title("Result")
    timing = re.search(r"S2 timing: cycles=(\d+) max_exec_us=(\d+) max_late_us=(\d+)", text)
    for tid in UNIT_IDS:
        hit = re.search(re.escape(tid) + r" (PASS|FAIL|SKIP)", text)
        note = ""
        if tid == "S2-U06" and timing:
            note = "WCET %s us, jitter %s us" % (timing.group(2), timing.group(3))
        if tid == "S2-U10" and hit and hit.group(1) == "SKIP":
            note = "Safety Monitor bound; covered by INT tests"
        if hit:
            result(tid, hit.group(1), rel(r.logpath), note)
        else:
            result(tid, "FAIL", rel(r.logpath), "line missing from output")
    if timing:
        say("")
        say("Measured: worst cycle time %s us (budget 300 us), worst lateness %s us (limit 2000 us)."
            % (timing.group(2), timing.group(3)))
        say("Write these into your Week 13 timing evidence.")
    say("Log saved: %s" % rel(r.logpath))


def t_m01(args):
    title("S2-M01  Encoder type: single channel or quadrature")
    step(1, "Power the Pico (USB is enough; any firmware). Motors off.")
    step(2, "Multimeter on DC volts. Black probe on GND.")
    step(3, "Red probe on GP14 (left A). Turn the left wheel VERY slowly by hand.")
    print("""
        quadrature              single channel
     A  _|~|_|~|_|~|_        A  _|~|_|~|_|~|_
     B  __|~|_|~|_|~|        B  ______________   (never changes)
    """)
    a_ok = yes("Does GP14 jump between about 0 V and 3.3 V as you turn?")
    if not a_ok:
        bad("left channel A dead: check the A wire, 3V3 and GND to the encoder.")
        result("S2-M01", False, "multimeter", "left A did not toggle")
        return
    step(4, "Red probe on GP15 (left B). Turn slowly again.")
    b_ok = yes("Does GP15 also jump between 0 V and 3.3 V?", "n")
    step(5, "Repeat on the right encoder: GP4 (A) then GP5 (B).")
    ra = yes("Does GP4 toggle?")
    rb = yes("Does GP5 toggle?", "n")
    if not ra:
        bad("right channel A dead: check the Grove 3 cable.")
        result("S2-M01", False, "multimeter", "right A did not toggle")
        return
    quad = b_ok and rb
    if b_ok != rb:
        warn("left and right differ - check the B wire on the side that did not toggle.")
    kind = "quadrature (A+B)" if quad else "single channel"
    result("S2-M01", "PASS", "multimeter", "encoder type: " + kind)
    calibrate(F_HDR, "S2_ENC_QUADRATURE", 1 if quad else 0, kind)


def signed16(v):
    v &= 0xFFFF
    return v - 0x10000 if v >= 0x8000 else v

def um_per_pulse_now():
    d = int(read_define(F_HDR, "S2_WHEEL_DIAMETER_MM"))
    n = int(read_define(F_HDR, "S2_ENC_PULSES_PER_REV"))
    # same integer maths as S2_UM_PER_PULSE in motion_math.h
    return (d * 314159 * 10) // (n * 100000 // 100), d, n

def t_m02(args):
    title("S2-M02  Pulses per wheel turn and wheel diameter")
    step(1, "Measure the wheel diameter with a ruler (across the tyre).")
    dia = ask_num("Wheel diameter in mm?", read_define(F_HDR, "S2_WHEEL_DIAMETER_MM"))
    step(2, "Put a tape mark on each wheel. Motors may stay connected (no drive).")
    enter()
    if not prepare(args, {"S2_TEST_DRIVE": 0, "S2_LOG_EVERY_CYCLES": 25}):
        return
    r = open_reader(args, "pulses")
    say("Waiting for the distance log ...")
    t0 = time.time()
    while r.last_row is None and time.time() - t0 < 60:
        try:
            show(r.lines.get(timeout=0.5))
        except queue.Empty:
            pass
    if r.last_row is None:
        r.close()
        bad("no S2,... lines.  Was the new firmware flashed?")
        return
    um, d_now, n_now = um_per_pulse_now()
    found = []
    for wheel in ("LEFT", "RIGHT"):
        step(3 if wheel == "LEFT" else 4, "Hold the %s wheel still. Mark at the top." % wheel)
        enter("Press Enter, then turn the %s wheel exactly 10 turns forward" % wheel)
        d0 = r.last_row["dist"]
        say("Turn now.  Press Enter after exactly 10 turns.")
        stop = threading.Event()
        def ticker():
            while not stop.is_set():
                sys.stdout.write("\r      distance change: %5d mm   " %
                                 abs(signed16(r.last_row["dist"] - d0)))
                sys.stdout.flush()
                time.sleep(0.5)
        th = threading.Thread(target=ticker, daemon=True)
        th.start()
        try:
            input()
        except EOFError:
            pass
        time.sleep(0.6)  # let the last log line arrive
        stop.set()
        delta = signed16(r.last_row["dist"] - d0)
        if delta < 0:
            warn("%s counted BACKWARDS while you turned it forward." % wheel)
            if read_define(F_HDR, "S2_ENC_QUADRATURE") == "1":
                calibrate(F_HDR, "S2_ENC_%s_INVERT" % wheel[0], "TRUE", "A/B swapped")
            delta = -delta
        # the log shows the average of both wheels, so one wheel shows half
        pulses = delta * 2 * 1000.0 / um
        ppr = pulses / 10.0
        print()
        say("%s: distance change %d mm -> about %.0f pulses in 10 turns -> %.1f per turn"
            % (wheel, delta, pulses, ppr))
        if delta == 0:
            bad("no pulses counted: encoder A not reaching the Pico. Redo S2-M01.")
        found.append(ppr)
    r.close()
    if not found or min(found) <= 0:
        result("S2-M02", False, rel(r.logpath), "no pulses on one wheel")
        return
    ppr = round(sum(found) / len(found))
    if abs(found[0] - found[1]) > max(1.5, 0.03 * ppr):
        warn("left %.1f and right %.1f per turn differ - nothing saved." % tuple(found))
        say("Repeat this test; you may have miscounted turns or one encoder misses slots.")
        result("S2-M02", False, rel(r.logpath), "L %.1f vs R %.1f pulses/turn" % tuple(found))
        return
    mm_pulse = math.pi * dia / ppr
    track = float(read_define(F_HDR, "S2_TRACK_WIDTH_MM"))
    say("Result: %d pulses per turn, %.1f mm per pulse, about %.0f deg of turn per pulse"
        % (ppr, mm_pulse, mm_pulse / (math.pi * track) * 360))
    result("S2-M02", "PASS", rel(r.logpath),
           "%d pulses/turn (L %.1f, R %.1f), wheel %d mm" % (ppr, found[0], found[1], dia))
    calibrate(F_HDR, "S2_ENC_PULSES_PER_REV", ppr, "measured")
    calibrate(F_HDR, "S2_WHEEL_DIAMETER_MM", int(round(dia)), "measured")
    step(5, "Measure the track width: centre of the left tyre to centre of the right tyre.")
    tw = ask_num("Track width in mm? (Enter to skip)", "", allow_blank=True)
    if tw:
        calibrate(F_HDR, "S2_TRACK_WIDTH_MM", int(round(float(tw))), "measured")


def drive_run(args, tag, log_every):
    """Build/flash with S2_TEST_DRIVE 1, wait for the M05 result line."""
    if not prepare(args, {"S2_TEST_DRIVE": 1, "S2_LOG_EVERY_CYCLES": log_every}):
        return None, None
    r = open_reader(args, tag)
    say("Waiting for the unit tests, then the wheels spin for 2 s ...")
    m = wait_for(r, r"S2-M05 speed L=(-?\d+) R=(-?\d+) mm/s, distance=(\d+)", 120,
                 quiet=False)
    time.sleep(1.0)  # catch the stop in the log
    r.close()
    return r, m

def t_m03(args):
    title("S2-M03 + S2-M05  Direction and speed, wheels up")
    step(1, "Connect both motors (M1 left, M2 right) and both encoders. Battery ready.")
    danger_wheels_up()
    enter()
    while True:
        r, m = drive_run(args, "drive", 0)
        if r is None:
            return
        if not m:
            bad("no 'S2-M05 speed' line.  Log: %s" % rel(r.logpath))
            return
        sl, sr, dist = int(m.group(1)), int(m.group(2)), int(m.group(3))
        say("")
        say("Measured: left %d mm/s, right %d mm/s, distance %d mm" % (sl, sr, dist))
        step(2, "What did the wheels do? (forward = the way the robot would drive)")
        say("1  both forward")
        say("2  LEFT ran backwards")
        say("3  RIGHT ran backwards")
        say("4  both ran backwards")
        say("5  a wheel did not turn at all")
        a = ask("Choose 1-5", "1")
        if a in ("2", "3", "4"):
            if a in ("2", "4"):
                calibrate(F_HDR, "S2_MOTOR_L_INVERT", "TRUE", "left ran backwards")
            if a in ("3", "4"):
                calibrate(F_HDR, "S2_MOTOR_R_INVERT", "TRUE", "right ran backwards")
            result("S2-M03", False, rel(r.logpath), "direction wrong, invert flag set")
            if yes("Run again with the fix?"):
                continue
            return
        if a == "5":
            result("S2-M03", False, rel(r.logpath), "a wheel did not turn")
            say("Check motor power and the M1/M2 plug.  If it hums but does not turn,")
            say("raise S2_FF_DEADBAND_PERMILLE in motion_math.h by 50.")
            return
        result("S2-M03", "PASS", rel(r.logpath), "both wheels forward")
        for side, v in (("left", sl), ("right", sr)):
            if v == 0:
                warn("%s speed 0 although it turned: encoder signal missing (redo S2-M01)." % side)
        ok = 180 <= sl <= 220 and 180 <= sr <= 220
        result("S2-M05", ok, rel(r.logpath), "L %d, R %d mm/s (target 180-220)" % (sl, sr))
        if not ok:
            say("Speeds off target: check S2-M02 values first, then run the tuning test.")
        return


def analyse(rows, side):
    sp = "sp_" + side
    spd = "spd_" + side
    duty = "duty_" + side
    drive = [x for x in rows if x["state"] == 1 and x[sp] > 0]
    if len(drive) < 10:
        return None
    t0 = drive[0]["t"]
    target = max(x[sp] for x in drive)
    t_full = next(x["t"] for x in drive if x[sp] >= target)
    peak = max(x[spd] for x in drive)
    rise = next((x["t"] - t0 for x in drive if x[spd] >= 0.9 * target), None)
    band = 0.1 * target
    settle = None
    for i, x in enumerate(drive):
        if all(abs(y[spd] - target) <= band for y in drive[i:]) and x["t"] >= t_full:
            settle = x["t"] - t0
            break
    tail = [x for x in drive if x["t"] >= drive[-1]["t"] - 500]
    mean_tail = sum(x[spd] for x in tail) / len(tail)
    crossings = sum(1 for a, b in zip(tail, tail[1:]) if (a[spd] - target) * (b[spd] - target) < 0)
    tail_duty = sum(abs(x[duty]) for x in tail) / len(tail)
    first_duty = max(abs(x[duty]) for x in drive[:25])      # highest duty in the first 0.5 s
    return dict(target=target, peak=peak, rise=rise, settle=settle, mean_tail=mean_tail,
                tail_duty=tail_duty, first_duty=first_duty,
                overshoot=100.0 * (peak - target) / target, crossings=crossings,
                max_duty=max(abs(x[duty]) for x in drive))

def verdict(a):
    if a is None:
        return "no data", "FAIL"
    if a["peak"] == 0:
        return "speed always 0: encoder signal missing", "FAIL"
    if a["overshoot"] > 20 and a["first_duty"] > 1.3 * a["tail_duty"]:
        return ("OVERSHOOTS, then creeps down: feed-forward too strong (up to %.0f%% duty at the start, "
                "only needs %.0f%%). Lower S2_FF_DEADBAND_PERMILLE / S2_FF_GAIN_MILLI"
                % (a["first_duty"] / 10, a["tail_duty"] / 10)), "FAIL"
    if a["overshoot"] > 20 or a["crossings"] >= 4:
        return "WOBBLES: lower S2_KP_MILLI by ~30%; still wobbling -> halve S2_KI_MILLI", "FAIL"
    if a["mean_tail"] < 0.9 * a["target"]:
        if a["max_duty"] >= 990:
            return "STAYS BELOW at full power: motors cannot reach this speed (battery low?)", "FAIL"
        return "STAYS BELOW: raise S2_KI_MILLI (e.g. 60 -> 100)", "FAIL"
    if a["settle"] is None or a["settle"] > 1000:
        return "TOO SLOW: raise S2_KP_MILLI by ~30% or raise S2_FF_GAIN_MILLI", "FAIL"
    return "GOOD: settles within +/-10% in under 1 s", "PASS"

def save_csv(rows, path):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("t_ms,state,sp_l,spd_l,duty_l,sp_r,spd_r,duty_r,dist_mm\n")
        for x in rows:
            f.write("%d,%d,%d,%d,%d,%d,%d,%d,%d\n" % (x["t"], x["state"], x["sp_l"], x["spd_l"],
                    x["duty_l"], x["sp_r"], x["spd_r"], x["duty_r"], x["dist"]))

def plot(rows, path):
    try:
        import matplotlib
        import matplotlib.pyplot as plt
    except ImportError:
        say("(Install matplotlib for a picture:  py -m pip install matplotlib.  "
            "Or open the CSV in Excel.)")
        return
    t0 = rows[0]["t"]
    t = [(x["t"] - t0) / 1000.0 for x in rows]
    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    for ax, side, name in ((axs[0], "l", "Left"), (axs[1], "r", "Right")):
        ax.plot(t, [x["sp_" + side] for x in rows], "--", color="grey", label="setpoint")
        ax.plot(t, [x["spd_" + side] for x in rows], color="tab:blue", label="speed")
        ax2 = ax.twinx()
        ax2.plot(t, [x["duty_" + side] / 10.0 for x in rows], color="tab:orange",
                 alpha=0.5, label="duty %")
        ax2.set_ylabel("duty %")
        ax.set_ylabel("%s mm/s" % name)
        ax.legend(loc="upper left")
    axs[1].set_xlabel("time (s)")
    fig.suptitle("S2-M04 step response")
    fig.tight_layout()
    fig.savefig(str(path), dpi=110)
    say("Plot saved: %s (close the window to continue)" % rel(path))
    plt.show()

def gains():
    return {k: read_define(F_MATH, k) for k in
            ("S2_KP_MILLI", "S2_KI_MILLI", "S2_FF_GAIN_MILLI", "S2_FF_DEADBAND_PERMILLE")}

def suggest_ff(results):
    """Feed-forward that gives about 90% of the measured steady duty."""
    duties = [a["tail_duty"] for a in results if a]
    targets = [a["target"] for a in results if a]
    if not duties:
        return None
    d = max(duties) * 0.9
    dead = int(round(d * 0.5 / 10.0) * 10)
    gain = int(round((d - dead) * 1000.0 / max(targets) / 10.0) * 10)
    return dead, max(gain, 0)

def t_m04(args):
    title("S2-M04  PID tuning: step 0 -> 200 mm/s, wheels up")
    step(1, "Same setup as the direction test. Run that one first.")
    danger_wheels_up()
    enter()
    while True:
        g = gains()
        say("Current gains: " + ", ".join("%s %s" % kv for kv in g.items()))
        r, m = drive_run(args, "tune", 1)
        if r is None:
            return
        rows = r.rows
        if len(rows) < 20:
            bad("too few S2,... lines logged.")
            return
        csv = r.logpath.with_suffix(".csv")
        save_csv(rows, csv)
        title("Result")
        overall = True
        notes = []
        found = []
        for side, name in (("l", "Left"), ("r", "Right")):
            a = analyse(rows, side)
            found.append(a)
            text, res = verdict(a)
            if a:
                say("%s: rise to 90%% %s ms, overshoot %.0f%%, settled %s ms, average at end %.0f mm/s"
                    % (name, a["rise"], a["overshoot"], a["settle"], a["mean_tail"]))
                notes.append("%s settle %s ms, overshoot %.0f%%" % (name[0], a["settle"], a["overshoot"]))
            (good if res == "PASS" else bad)("%s  %s" % (name, text))
            overall = overall and res == "PASS"
        say("CSV saved: %s" % rel(csv))
        plot(rows, csv.with_suffix(".png"))
        note = "; ".join(notes) + "; KP %s KI %s FF %s" % (g["S2_KP_MILLI"], g["S2_KI_MILLI"],
                                                           g["S2_FF_GAIN_MILLI"])
        result("S2-M04", overall, rel(csv), note)
        if overall:
            say("Keep these gains.  Repeat this test later on the floor (wheels loaded).")
            return
        ff = suggest_ff(found)
        if ff and any(a and a["overshoot"] > 20 and a["first_duty"] > 1.3 * a["tail_duty"] for a in found):
            say("Suggested feed-forward from this run: S2_FF_DEADBAND_PERMILLE %d, S2_FF_GAIN_MILLI %d"
                % ff)
            if yes("Use these and run again?"):
                for k, v in zip(("S2_FF_DEADBAND_PERMILLE", "S2_FF_GAIN_MILLI"), ff):
                    if str(v) != g[k]:
                        write_define(F_MATH, k, v)
                        CHANGES.append("%s %s -> %s" % (k, g[k], v))
                continue
        if not yes("Change a gain and run again?"):
            return
        for k in ("S2_KP_MILLI", "S2_KI_MILLI", "S2_FF_GAIN_MILLI", "S2_FF_DEADBAND_PERMILLE"):
            v = ask("New %s (Enter keeps %s)" % (k, g[k]), g[k])
            if v != g[k]:
                write_define(F_MATH, k, int(v))
                CHANGES.append("%s %s -> %s" % (k, g[k], v))


def t_m10(args):
    title("S2-M10  Stall detection, wheels up")
    step(1, "Same setup as the direction test.")
    step(2, "Before the Pico boots, PINCH THE LEFT TYRE firmly so it cannot turn.")
    say("Keep holding until the script says you can let go (about 10 s after boot).")
    say("The motor will push for about 0.3 s, then must stop by itself.")
    danger_wheels_up()
    enter()
    r, m = drive_run(args, "stall", 1)
    if r is None:
        return
    say("You can let go now.")
    rows = r.rows
    drive = [x for x in rows if x["sp_l"] > 0 or x["state"] in (1, 6)]
    start = next((x for x in rows if x["state"] == 1 and x["sp_l"] > 0), None)
    stall = next((x for x in rows if x["state"] == 6), None)
    if start is None:
        bad("the drive never started.  Log: %s" % rel(r.logpath))
        return
    if stall is None:
        held = [x for x in rows if x["state"] == 1 and x["sp_l"] >= 60]
        states = sorted(set(STATE.get(x["state"], str(x["state"])) for x in rows))
        result("S2-M10", False, rel(r.logpath), "state STALLED never reached")
        if held:
            say("While driving: left duty up to %d%%, left speed %d..%d mm/s, states seen: %s"
                % (max(abs(x["duty_l"]) for x in held) / 10,
                   min(x["spd_l"] for x in held), max(x["spd_l"] for x in held),
                   ", ".join(states)))
            if max(abs(x["duty_l"]) for x in held) < 400:
                say("Duty never reached 40%, so stall detection could not start: the")
                say("motor was not pushing hard against your fingers.  Hold it from boot.")
            else:
                say("Duty was high but the left speed reading stayed up: the wheel was still")
                say("creeping.  Grip the tyre harder, or press the wheel against the box.")
        return
    after = [x for x in rows if x["t"] > stall["t"]]
    off = all(x["duty_l"] == 0 and x["duty_r"] == 0 for x in after)
    dt = stall["t"] - start["t"]
    ok = off and dt <= 500
    result("S2-M10", ok, rel(r.logpath),
           "STALLED after %d ms, motors %s" % (dt, "off" if off else "NOT off"))
    say("Expected: about 300 ms after the motor reaches the stall duty (plus the soft start).")


def t_m08(args):
    title("S2-M08  Left and right turns on the spot")
    if read_define(F_TEST, "S2_TEST_TURN") is None:
        bad("subsystem2_test.c has no S2_TEST_TURN switch: copy in the latest subsystem2_test.c.")
        return
    step(1, "Where do you want to test?")
    say("1  WHEELS UP: only checks which way each wheel turns (quick, safe)")
    say("2  ON THE FLOOR: measures the angles (the real S2-M08 test)")
    floor = ask("Choose 1 or 2", "1") == "2"
    if floor:
        step(2, "Floor setup:")
        say("- Clear 50 cm around the robot, on a flat floor (not thick carpet).")
        say("- Stick a strip of tape on the floor along the robot's centre line, front to back.")
        say("- Keep the USB cable slack so it doesn't pull the robot round.")
        say("- After each turn, measure the angle between the robot and the tape")
        say("  (protractor, or a phone compass/level app), then re-line it up.")
    else:
        danger_wheels_up()
    say("The robot makes 4 turns: right 90, left 90, right 180, left 180, 10 s apart.")
    enter()
    if not prepare(args, {"S2_TEST_DRIVE": 0, "S2_TEST_TURN": 1, "S2_LOG_EVERY_CYCLES": 0}):
        return
    r = open_reader(args, "turns")
    say("Waiting for the unit tests (about 5 s), then 8 s to put the robot down ...")
    results = []
    for i, deg in enumerate((90, -90, 180, -180)):
        m = wait_for(r, r"S2-M08 turn (-?\d+) deg (done in (\d+) ms|NOT finished)", 90)
        if not m:
            bad("no turn result line.  Log: %s" % rel(r.logpath))
            break
        finished = m.group(3) is not None
        side = "RIGHT (clockwise)" if deg > 0 else "LEFT (anticlockwise)"
        if not finished:
            bad("turn %+d did not finish within 6 s" % deg)
            say("A wheel may not be counting (check its encoder) or is slipping/stalled.")
            results.append((deg, None, False))
            continue
        if floor:
            v = ask_num("Turn %d was %s %d deg. How many degrees did it actually turn?"
                        % (i + 1, side, abs(deg)), "", allow_blank=True)
            if v is None:
                results.append((deg, None, True))
                continue
            err = v - abs(deg)
            ok = abs(err) <= 5
            (good if ok else bad)("turn %+d: measured %.0f deg, error %+.0f deg (limit +/-5)"
                                  % (deg, v, err))
            results.append((deg, v, ok))
            say("Line the robot up with the tape again - next turn in about 10 s.")
        else:
            want = "LEFT wheel forward, RIGHT wheel backward" if deg > 0 \
                else "RIGHT wheel forward, LEFT wheel backward"
            ok = yes("Turn %d (%s): did the %s?" % (i + 1, side, want))
            if not ok:
                say("Check S2-M03 first (both wheels forward on the drive test).")
                say("If that passed, the encoder of one wheel counts backwards: run option 4,")
                say("which offers to set S2_ENC_L_INVERT / S2_ENC_R_INVERT.")
            results.append((deg, None, ok))
    wait_for(r, r"S2-M08 all turns sent", 15, quiet=True)
    r.close()
    if not results:
        return
    all_ok = all(ok for _, _, ok in results) and len(results) == 4
    if floor:
        measured = ["%+d:%s" % (d, ("%.0f" % v) if v is not None else "-") for d, v, _ in results]
        result("S2-M08", all_ok, rel(r.logpath), "floor, measured " + ", ".join(measured))
        say("The test plan asks for 10 of each turn: run this option a few more times.")
    else:
        result("S2-M08", "PASS" if all_ok else "FAIL", rel(r.logpath),
               "wheels up: direction only" + ("" if all_ok else ", wrong direction"))
        if all_ok:
            say("Directions are right.  Now run it again ON THE FLOOR to measure the angles.")


def t_record(args):
    title("Record results in TEST_LOG.md")
    if not RESULTS:
        say("Nothing measured yet in this session.")
        return
    restore_switches()   # so the debug switches do not count as changes
    for tid, res, ev, note in RESULTS:
        say("%-7s %-5s %-38s %s" % (tid, res, ev, note))
    if CHANGES:
        say("")
        say("Calibration changed in this session: " + "; ".join(CHANGES))
    try:
        commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=str(REPO),
                                capture_output=True, text=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain", "app_program"], cwd=str(REPO),
                               capture_output=True, text=True).stdout.strip()
    except FileNotFoundError:
        commit, dirty = "", ""
    suffix = ""
    if dirty:
        warn("app_program has uncommitted changes, so commit %s is not exactly what you tested." % commit)
        say("Best: quit, commit your calibration changes, re-run the tests, then record.")
        if not yes("Record anyway, marked 'uncommitted changes'?", "n"):
            return
        suffix = " (tested with uncommitted changes)"
    tester = ask("Tester name?", "Aaron")
    today = datetime.date.today().isoformat()
    rows = []
    for tid, res, ev, note in RESULTS:
        rows.append("| %s | %s | `%s` | %s | %s | `%s` | %s |"
                    % (today, tid, commit or "?", tester, res, ev, (note + suffix).strip()))
    with open(TEST_LOG, encoding="utf-8", newline="") as f:
        text = f.read()     # keep the file's own line endings
    nl = "\r\n" if "\r\n" in text else "\n"
    if not text.endswith(("\n", "\r\n")):
        text += nl
    text += nl.join(rows) + nl
    with open(TEST_LOG, "w", encoding="utf-8", newline="") as f:
        f.write(text)
    good("%d rows added to %s" % (len(rows), TEST_LOG.relative_to(REPO).as_posix()))
    say("Next:  git add testing/subsystem2  &&  git commit -m \"S2: bench test results\"")
    RESULTS.clear()


# ================================================================ menu
MENU = [
    ("1", "PC maths tests (no Pico)                 S2-H01..H12", t_host),
    ("2", "Automatic tests on the Pico (no motion)  S2-U01..U11", t_unit),
    ("3", "Encoder type (multimeter)                S2-M01", t_m01),
    ("4", "Pulses per turn (turn by hand)           S2-M02", t_m02),
    ("5", "Direction + speed, WHEELS UP             S2-M03, M05", t_m03),
    ("6", "PID tuning with plot, WHEELS UP          S2-M04", t_m04),
    ("7", "Stall detection, WHEELS UP               S2-M10", t_m10),
    ("8", "Left / right turns                       S2-M08", t_m08),
    ("9", "Write results into TEST_LOG.md", t_record),
]

def main():
    ap = argparse.ArgumentParser(description="Guided bench tests for Subsystem 2")
    ap.add_argument("--port", help="serial port, e.g. COM5 (default: find the Pico)")
    ap.add_argument("--no-build", action="store_true", help="do not build/flash automatically")
    args = ap.parse_args()

    if not F_TEST.exists():
        print("Run this from the repository on the s2-motion-control branch.")
        sys.exit(1)
    if not IS_WIN:
        warn("Not running on Windows.  Inside WSL the Pico's COM port is not visible;")
        say("run this in PowerShell with  py testing\\subsystem2\\s2_bench.py  instead.")

    try:
        while True:
            title("Subsystem 2 bench tests  -  do them in order the first time")
            for key, label, _ in MENU:
                print("   %s  %s" % (key, label))
            print("   q  quit (debug switches are put back)")
            if RESULTS:
                say(c("2", "%d result(s) not yet written to TEST_LOG.md (option 9)" % len(RESULTS)))
            choice = ask("Choose")
            if choice.lower() == "q":
                if RESULTS and yes("Write the %d result(s) to TEST_LOG.md first?" % len(RESULTS)):
                    t_record(args)
                break
            for key, _, fn in MENU:
                if choice == key:
                    try:
                        fn(args)
                    except KeyboardInterrupt:
                        print()
                        warn("test stopped (Ctrl+C).  Back to the menu.")
                    break
    except KeyboardInterrupt:
        print()
    finally:
        restore_switches()
        if CHANGES:
            print(c("1", "\n  Calibration changes kept in the source: ") + "; ".join(CHANGES))
            print("  Rebuild before your next test and commit them on your branch.")


if __name__ == "__main__":
    main()
