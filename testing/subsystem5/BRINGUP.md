# Subsystem 5 bring-up: one phase at a time

Goal: never connect or drive anything that has not been checked first.
Change `S5_PHASE` in `app_program/subsystem5/s5_phase.h`, rebuild, flash,
read the console, tick the check, commit, THEN go to the next phase.

Rules that protect the board
- Unplug USB and battery before you add, move or remove ANY wire.
- Change one thing per phase. If something looks wrong, stop and unplug.
- The Pico GPIO pins take 3.3 V maximum. They are NOT 5 V tolerant.
- Commit your code BEFORE you test it, and write the commit hash in the
  Results table of `TEST_LOG.md`.

| Phase | Wired | Pins the firmware touches | Check |
|---|---|---|---|
| 0 | nothing (PC only) | none | `sh testing/subsystem5/host/run.sh` prints `20 passed, 0 failed` |
| 1 | nothing | none | Console shows `s5 phase 1: alive`, `PWM out of reset = 1` |
| 2 | servo on servo port 1 | GP12 | Servo centres, then sweeps 60 to 120 degrees slowly and smoothly |
| 3 | + HC-SR04 VCC, GND, TRIG | GP12, GP16 | Console counts triggers; ECHO still NOT connected |
| 4 | + ECHO through the divider | GP12, GP16, GP17 | `echo` is about 58 us per cm; moving a card changes it |
| 5 | same wiring | same | Full scan prints `PROFILE ...` lines; log scan time and width error |
| 6 | later | later | Vehicle Controller drives it |

## Phase 0: PC only

1. Open the repo folder in a terminal (WSL or Git Bash).
2. `sh testing/subsystem5/host/run.sh`
3. Expect: `[HOST] subsystem5 SUMMARY 20 passed, 0 failed`. Save the output
   to `testing/subsystem5/logs/<date>_host.txt`.

## Phase 1: board only, nothing wired

1. In `s5_phase.h` make sure `#define S5_PHASE 1`.
2. WSL: `cd build_make && make -j8` (see README section 1 and 2 for setup).
3. Hold BOOTSEL, plug in USB, copy `build_make/mtk3pico_smp1_usb_cdc_wifi.uf2`
   onto the RPI-RP2 drive.
4. Open the serial port (PuTTY or the VS Code Serial Monitor, 115200).
5. Expect `s5 bring-up PHASE 1` then `s5 phase 1: alive N s, PWM out of reset = 1`.
   If it says 0, stop: tell Chong/Claude, do not wire anything.
6. Optional: build the test image `make TEST=1 -j8`, flash it, and expect
   `[TEST] S5-U01 ... U09 PASS` (U03 and U04 only appear from phase 2).

## Phase 2: servo only

Before wiring
- The servo port takes its power from the board's POWER INPUT (battery),
  not from USB. Measure that voltage with a multimeter. An SG90 is happy
  at 4.8 to 6 V. Do not connect a servo if it is higher.
- The servo plug: brown/black = GND, red = V+, orange/yellow = signal.
  The signal pin is the one nearest the board edge labelled GP12 /
  servo port 1. Check the board's silkscreen, not memory.

Steps
1. Unplug everything. Plug the servo into servo port 1 WITHOUT the horn.
2. `S5_PHASE 2`, build, flash. Power the board as in your datasheet.
3. The servo goes to 90 degrees and holds for 5 s. Fit the horn now so
   it points straight ahead (the middle of the scan).
4. It then sweeps 60 to 120 degrees. If it buzzes, strains or hits a stop,
   unplug at once.
5. Pass: smooth sweep, centred when the horn points ahead. If the sweep
   goes the wrong way round for your mounting, set
   `S5_SERVO_REVERSED` to 1 in `drv_ultrasonic.h`.
6. When happy, widen `S5_SWEEP_MIN/MAX` towards 30 and 150 in
   `s5_bringup.c`, never past what the servo reaches without straining.

## Phase 3: add the HC-SR04 supply and TRIG only

The Robo Pico Grove ports are 3.3 V. A normal HC-SR04 wants 5 V. Do not
guess: decide the supply first.
- If the module is powered from 3.3 V: ECHO is about 3.3 V. Still use the
  divider if you are unsure.
- If it is powered from 5 V (or from the servo rail, which equals the
  battery voltage and may be above 5.5 V): ECHO goes up to 5 V, so a
  1k / 2k divider (or a level shifter) is REQUIRED.

Test the divider BEFORE it touches the Pico
1. Build: ECHO -> 1 kohm -> point P -> 2 kohm -> GND. P goes to GP17.
2. With the Pico UNPLUGGED, apply 5 V from a bench supply or a USB 5 V
   source to the 1 kohm input only, GND common.
3. Measure P to GND: expect 3.3 V (3.2 to 3.4 V). Anything above 3.4 V:
   stop, check the resistor values.
4. Leave P NOT connected to GP17 for this phase.

Steps
1. Unplug everything. Wire VCC and GND to the module, TRIG to GP16.
   ECHO stays unconnected to the Pico.
2. `S5_PHASE 3`, build, flash. The console counts triggers twice a second.
3. Pass: no crash, no heat, module stays cool.

## Phase 4: connect ECHO through the divider

1. Unplug everything. Connect P (divider midpoint) to GP17. Double-check
   with the divider test result from phase 3.
2. `S5_PHASE 4`, build, flash.
3. Hold a flat card at 10, 20, 50, 100 cm. Expect about 580, 1160, 2900,
   5800 us. Log the table (ruler cm vs reported cm). Test IDs S5-M03, S5-M04.
4. Point at open space: note whether you get `status 1` (about 38000 us,
   normal) or `status 2` (no pulse). That decides `S5_NOECHO_FAULT_N`
   (test S5-M06).
5. Unplug the sensor: expect `status 2` (test S5-M05).

## Phase 5: full scan

1. `S5_PHASE 5`, build, flash. The robot stays still; put a box 30 cm in front.
2. Expect `obstacle: ... deg` lines then
   `obstacle: PROFILE seq N detected 1 closest .. cm @.. deg, width .. cm, left .., right .., action ..`.
3. Record scan time (S5-M07) and width error against known objects (S5-M08).

## Git: your own branch

See the chat instructions or `git switch -c` below.

```
git fetch origin
git branch -r                     # see how your teammates named theirs
git switch main
git pull
git switch -c s5-obstacle-detection     # use the same style as theirs
# copy the new files in, then:
git status
git add app_program/subsystem5 testing/subsystem5
git commit -m "S5: obstacle detection driver, scan maths, phased bring-up"
git push -u origin s5-obstacle-detection
```

After each phase passes:
```
git add testing/subsystem5
git commit -m "S5: phase N passed (log + results)"
git push
```
