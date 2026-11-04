# Subsystem 5 — test log

**Subsystem:** Spatial Optimizer (Obstacle Detection, Vehicle Controller, Safety Monitor)  ·  **Student ID:** 2503437  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results. Hardware bring-up is done one phase at a time, see `BRINGUP.md`.

## Test cases

Define a test here before running it. Test IDs never change.

### Automatic, on a PC (no Pico)

Run from the repository root (WSL or Linux): `sh testing/subsystem5/host/run.sh`

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-H01 | echo time to distance, 58 µs per cm, rounded | NFR27 | auto (PC) | `host/test_profile.c` | PASS |
| S5-H02 | servo pulse 600 / 1500 / 2400 µs for 0 / 90 / 180°, clamped above 180° | FR38 | auto (PC) | as above | PASS |
| S5-H03 | integer square root | FR40 | auto (PC) | as above | PASS |
| S5-H04 | clear scan: not detected, action CONTINUE | FR38 | auto (PC) | as above | PASS |
| S5-H05 | cluster of hits found, closest point reported | FR40 | auto (PC) | as above | PASS |
| S5-H06 | width of a narrow obstacle at 30 cm is about 8 cm (7.8 expected) | NFR28 | auto (PC) | as above | PASS |
| S5-H07 | symmetric obstacle: equal clearances, tie goes right | FR41 | auto (PC) | as above | PASS |
| S5-H08 | obstacle on the right: left shift smaller, TURN_LEFT | FR41, NFR29 | auto (PC) | as above | PASS |
| S5-H09 | obstacle on the left: right shift smaller, TURN_RIGHT | FR41, NFR29 | auto (PC) | as above | PASS |
| S5-H10 | wall across all angles: partial, REVERSE_RETRY | FR41 | auto (PC) | as above | PASS |
| S5-H11 | closer than the stop distance: STOP | FR41 | auto (PC) | as above | PASS |
| S5-H12 | single sample: flagged partial, width = one beam footprint | FR40, A5 | auto (PC) | as above | PASS |
| S5-H13 | reading above the detect distance is ignored | FR38 | auto (PC) | as above | PASS |
| S5-H14 | fine scan walks outwards until an edge is found (a, b, c) | FR39 | auto (PC) | as above | PASS |
| S5-H15 | scan limit: no index below 0, next index is 1 | FR39 | auto (PC) | as above | PASS |
| S5-H16 | i8 profile message round-trips at every field limit | i8, minimum-bit rule | auto (PC) | as above | PASS |
| S5-H17 | unused bits of the i8 message stay zero (31 + 18 bits used) | i8, minimum-bit rule | auto (PC) | as above | PASS |
| S5-H18 | message sequence wraps in 4 bits | i8 | auto (PC) | as above | PASS |
| S5-H19 | median of repeated readings: outlier rejected, majority "nothing" wins | professor guidance (median filter) | auto (PC) | as above | PASS |
| S5-H20 | a reading equal to its background distance is not an obstacle | professor guidance (background subtraction) | auto (PC) | as above | PASS |
| S5-H21 | box in front of a wall: only the angles clearly closer than the wall are hits | professor guidance (background subtraction) | auto (PC) | as above | PASS |
| S5-H22 | hysteresis with a background: starts at 6 cm closer, stays in at 3 cm closer (a, b) | professor guidance (hysteresis) | auto (PC) | as above | PASS |
| S5-H23 | hysteresis without a background: start at 50 cm, stays in up to 55 cm (a, b) | professor guidance (hysteresis) | auto (PC) | as above | PASS |
| S5-H24 | a close background straight ahead (indices 5..7) is forgotten, so it can never hide a real obstacle (a, b) | safety | auto (PC) | as above | PASS |
| S5-H25 | bypass shifts are planned for at least a 20 cm obstacle; switching the planning off shrinks them (a, b, c, d) | professor guidance (maximum expected width) | auto (PC) | as above | PASS |
| S5-H26 | width uncertainty from the 15 degree step is reported (+/- 7 cm at 30 cm range) | professor guidance (report uncertainty) | auto (PC) | as above | PASS |
| S5-H27 | simulation: 390 flat-faced boxes (10 to 20 cm wide, 20 to 40 cm away, offset -30 to +30 cm, 15 degree beam, 1 cm rounding). Every chosen bypass must keep the robot's half width clear of the true edge and no box in the lane may be missed | professor guidance (consistently safe path) | auto (PC) | as above | PASS, 0 unsafe, 0 missed |
| S5-H28 | same simulation with the servo limited to 60 to 120 degrees (the window the car body allows). Unmeasured edges make the profile partial and the bypass wider | professor guidance (consistently safe path) | auto (PC) | as above | PASS, 0 unsafe, 0 missed |
| S5-H29 | same simulation with the servo limited to 45 to 135 degrees | professor guidance (consistently safe path) | auto (PC) | as above | PASS, 0 unsafe, 0 missed |
| S5-H30 | same simulation with the servo limited to 30 to 150 degrees (sensor moved forward so it clears the car body) | professor guidance (consistently safe path) | auto (PC) | as above | PASS, 0 unsafe, 0 missed |
| S5-H31 | hardware regression (2026-11-04 run): one box dead ahead and one to the right. The profile must turn LEFT. The earlier code looked only at the closest box and said turn right 17 cm, towards the second box | professor guidance (safe path), safety | auto (PC) | as above | PASS |
| S5-H32 | hardware regression: boxes on the right (30 to 60 deg) and on the left (120 deg). Neither side is free, so REVERSE_RETRY | safety | auto (PC) | as above | PASS |
| S5-H33 | hardware regression: one wide obstacle from 30 to 120 deg is a single cluster and gets a bypass | safety | auto (PC) | as above | PASS |
| S5-H34 | fine scan picks the next unmeasured neighbour of any obstacle inside the servo window, nearest to the servo first (a to d) | FR39 | auto (PC) | as above | PASS |
| S5-H35 | simulation with TWO boxes (10 to 20 cm wide, 980 placements, servo window 30 to 150 deg). Every chosen bypass clears both boxes, none missed | professor guidance (consistently safe path) | auto (PC) | as above | PASS, 0 unsafe, 0 missed |

### Automatic, on the Pico (`make TEST=1`)

None of these needs the sensor. With `S5_PHASE 1` (s5_phase.h) no pin is driven. S5-U03 and S5-U04 are only compiled from phase 2.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | runs in a task context | | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | PWM block is out of reset (the kernel leaves it in reset) | servo, echo timing | auto | `subsystem5_test.c` | `[TEST] S5-U02 PASS` |
| S5-U03 | servo slice 6 runs with a 20 ms period (phase 2 and up) | FR38 | auto | `subsystem5_test.c` | `[TEST] S5-U03 PASS` |
| S5-U04 | servo starts centred, 1500 µs pulse (phase 2 and up) | FR38 | auto | `subsystem5_test.c` | `[TEST] S5-U04 PASS` |
| S5-U05 | Obstacle state fits in 3 bits | minimum-bit rule | auto | `subsystem5_test.c` | `[TEST] S5-U05 PASS` |
| S5-U06 | i8 profile message packs and unpacks | i8 | auto | `subsystem5_test.c` | `[TEST] S5-U06 PASS` |
| S5-U07 | i8 message is 8 bytes | minimum-bit rule | auto | `subsystem5_test.c` | `[TEST] S5-U07 PASS` |
| S5-U08 | two simultaneous fault bits are both kept | i14 (IF-12) | auto | `subsystem5_test.c` | `[TEST] S5-U08 PASS` |
| S5-U09 | clearing one fault bit with TWF_BITCLR leaves the other set | i14 (IF-12) | auto | `subsystem5_test.c` | `[TEST] S5-U09 PASS` |

### Manual, on the bench

Do them in phase order (`BRINGUP.md`). Never wire the next part until the previous phase passes.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-M01 | phase 1: board runs with nothing wired, PWM out of reset | bring-up | manual | `S5_PHASE 1`, flash, read the console | `s5 phase 1: alive` lines with `PWM out of reset = 1` |
| S5-M02 | phase 2: servo centres, then sweeps 60 to 120° smoothly | FR38 | manual | `S5_PHASE 2`, servo on port 1, watch it | Smooth sweep, no buzzing, horn points ahead at 90° |
| S5-M03 | echo divider: midpoint voltage with 5 V applied, Pico UNPLUGGED | R5, Week 6 Section 8.3 | manual | Apply 5 V to the 1 kΩ input, measure the midpoint to GND with a multimeter | 3.2 to 3.4 V |
| S5-M04 | phase 3: HC-SR04 supply and TRIG only, ECHO not connected | R5 | manual | `S5_PHASE 3`, read the console | Triggers counted, module stays cool |
| S5-M05 | phase 4: raw echo is about 58 µs per cm | NFR27 | manual | `S5_PHASE 4`, flat card at 10, 20, 50, 100 cm | About 580 / 1160 / 2900 / 5800 µs |
| S5-M06 | distance against a ruler, 5 distances, 20 readings each; detection rate | NFR27 | manual | Table of ruler cm vs reported cm | Within 2 cm; at least 19 of 20 detected |
| S5-M07 | open space: does the kit's sensor send a pulse when nothing is in range? | S5_NOECHO_FAULT_N | manual | Point at open space | `status 1` (about 38000 µs) is normal. `status 2`: set `S5_NOECHO_FAULT_N` to 0 |
| S5-M08 | sensor unplugged: no echo is reported and the fault bit is raised | Version C row 3 | manual | Unplug the sensor | `status 2`, then `FAULT` after `S5_NOECHO_FAULT_N` readings |
| S5-M09 | phase 5: full scan time, detection to profile | R6, NFR31 | manual | Obstacle at 30 cm; read `scan N ms` on the PROFILE line | Total under 5 s target; record every run |
| S5-M10 | width error against known objects | NFR28, A5 | manual | Objects 5, 10 and 20 cm wide at 20 and 40 cm. Tune `beam_comp_deg` | Within 20% (target 10%) |
| S5-M11 | phase 5: two objects in the scan, both are profiled and the side of the second object is blocked | Professor feedback (safe navigation), NFR28 | manual | Reset with nothing in front, wait for the 9 `background` lines (no WARNING), then place one object about 30 cm ahead and a second about 30 cm to one side with a gap of at least 15 cm | `objects 2`; the side next to the second object reports `510 cm`; the action turns to the free side with clearance of at least 11 cm (half width 8 + margin 3) |
| S5-M12 | phase 5: repeatability, same setup scanned at least 3 times in a row | Professor feedback (repeated measurements), NFR28 | manual | Leave the setup untouched and read three consecutive PROFILE lines | `width`, `planned`, `left` and `right` each within 2 cm of each other, same action every time |
| S5-M13 | phase 5: background taken with an object in the lane ahead | Professor feedback (background subtraction) | manual | Put an object ahead before the reset | `WARNING ... background value(s) ahead were close and are ignored`, the close lane angles are stored as `511 cm (none)`, and an object placed there afterwards is still detected |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**auto (PC)** = runs on a PC, prints `[HOST]` lines;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-28 | S5-H01 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run, WSL, with address and undefined-behaviour checks |
| 2026-10-28 | S5-H02 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-H03 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-H04 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-H05 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-H06 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-H07 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-H08 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-H09 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-H10 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-H11 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-H12 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-H13 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-H14 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run, checks a, b and c |
| 2026-10-28 | S5-H15 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-H16 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-H17 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-H18 | `22b5879` | Min Han | PASS | `logs/2026-10-28_host.txt` | PC run |
| 2026-10-28 | S5-M01 | `22b5879` | Min Han | PASS | `logs/2026-10-28_S5-M01.txt` | Phase 1, normal image, COM3. `PWM out of reset = 1` on every `alive` line. Nothing wired. Log shows 0 to 30 s and 212 to 221 s; the middle of the capture was not saved |
| 2026-10-29 | S5-P01 | `22b5879` | Min Han | PASS | `logs/2026-10-28_S5-M01.txt` | Phase 1, normal image, COM3. `PWM out of reset = 1` on every `alive` line, nothing wired. Same run as the 2026-10-28 S5-M01 row, which used the wrong ID |
| 2026-10-29 | S5-M01 | `22b5879` | Min Han | VOID | `logs/2026-10-28_S5-M01.txt` | The 2026-10-28 row was logged under the wrong ID (that run was phase 1, S5-P01). The ECHO level test has not been run yet |
| 2026-10-29 | S5-U01 | `536e549` | Min Han | PASS | `logs/2026-10-29_S5-U01-U09.txt` | Test image, S5_PHASE 1, nothing wired, COM3 |
| 2026-10-29 | S5-U02 | `536e549` | Min Han | PASS | `logs/2026-10-29_S5-U01-U09.txt` | Test image, S5_PHASE 1 |
| 2026-10-29 | S5-U05 | `536e549` | Min Han | PASS | `logs/2026-10-29_S5-U01-U09.txt` | Test image, S5_PHASE 1 |
| 2026-10-29 | S5-U06 | `536e549` | Min Han | PASS | `logs/2026-10-29_S5-U01-U09.txt` | Test image, S5_PHASE 1 |
| 2026-10-29 | S5-U07 | `536e549` | Min Han | PASS | `logs/2026-10-29_S5-U01-U09.txt` | Test image, S5_PHASE 1 |
| 2026-10-29 | S5-U08 | `536e549` | Min Han | PASS | `logs/2026-10-29_S5-U01-U09.txt` | Test image, S5_PHASE 1 |
| 2026-10-29 | S5-U09 | `536e549` | Min Han | PASS | `logs/2026-10-29_S5-U01-U09.txt` | Test image, S5_PHASE 1 |
| 2026-11-01 | S5-P01 | `536e549` | Min Han | PASS | `logs/2026-11-01_S5-P01-U01-U09.txt` | Robo Pico board (the earlier run used a Maker Pi Pico). Test image, S5_PHASE 1, servo and HC-SR04 not connected. `PWM out of reset = 1` on every `alive` line in the log |
| 2026-11-01 | S5-U01 | `536e549` | Min Han | PASS | `logs/2026-11-01_S5-P01-U01-U09.txt` | Robo Pico, test image, S5_PHASE 1 |
| 2026-11-01 | S5-U02 | `536e549` | Min Han | PASS | `logs/2026-11-01_S5-P01-U01-U09.txt` | Robo Pico, test image, S5_PHASE 1 |
| 2026-11-01 | S5-U05 | `536e549` | Min Han | PASS | `logs/2026-11-01_S5-P01-U01-U09.txt` | Robo Pico, test image, S5_PHASE 1 |
| 2026-11-01 | S5-U06 | `536e549` | Min Han | PASS | `logs/2026-11-01_S5-P01-U01-U09.txt` | Robo Pico, test image, S5_PHASE 1 |
| 2026-11-01 | S5-U07 | `536e549` | Min Han | PASS | `logs/2026-11-01_S5-P01-U01-U09.txt` | Robo Pico, test image, S5_PHASE 1 |
| 2026-11-01 | S5-U08 | `536e549` | Min Han | PASS | `logs/2026-11-01_S5-P01-U01-U09.txt` | Robo Pico, test image, S5_PHASE 1 |
| 2026-11-01 | S5-U09 | `536e549` | Min Han | PASS | `logs/2026-11-01_S5-P01-U01-U09.txt` | Robo Pico, test image, S5_PHASE 1 |
| 2026-11-01 | S5-P02 | `e3b0f8a` | Min han | PASS | `logs/2026-11-01_S5-P02.txt` | Robo Pico, servo on port 1 (GP12), laptop USB power. Holds 90 deg for 5 s, then sweeps 60 to 120 smoothly. High angle = robot's left. Sensor sits off-centre on the horn (teeth steps), so S5_SERVO_TRIM_DEG = -3 makes 90 deg point straight ahead. Pulses 1170 to 1770 us |
| 2026-11-02 | S5-U03 | `482279b` | Min Han | PASS | `logs/2026-11-02_S5-U01-U09_phase2_nowifi.txt` | Robo Pico, S5_PHASE 2 test image built with WIFI=none, servo on GP12. INT-01 and INT-02 pass with the Wi-Fi radio off and failed with it on (earlier S5-U03 row), so that stall is the Wi-Fi start-up, not subsystem 5 |
| 2026-11-02 | S5-U04 | `482279b` | Min Han | PASS | `logs/2026-11-02_S5-U01-U09_phase2_nowifi.txt` | Same run as S5-U03. S5_SERVO_TRIM_DEG = -3 (expected pulse 1470 us) |
| 2026-11-01 | S5-U03 | `482279b` | Min Han | PASS | `logs/2026-11-01_S5-U01-U09_phase2.txt` | Robo Pico, S5_PHASE 2 test image, default build (Wi-Fi on), servo on GP12. In this run the S3/S4 example tests INT-01 and INT-02 failed. They fail on the phase 1 image too (`logs/2026-11-01_INT-stall_phase1.txt`) and pass with Wi-Fi off (2026-11-02 rows), so not caused by subsystem 5 |
| 2026-11-01 | S5-U04 | `482279b` | Min Han | PASS | `logs/2026-11-01_S5-U01-U09_phase2.txt` | Same run as the S5-U03 row above. S5_SERVO_TRIM_DEG = -3 (expected pulse 1470 us) |
| 2026-11-02 | S5-P03 | `4d784dc` | Min Han | N/A | - | Phase 3 (TRIG only) was skipped. The HC-SR04+ runs from 3.3 V on Grove 4 (GP16 TRIG, GP17 ECHO), so there is no divider and no 5 V step. Went straight to phase 4 |
| 2026-11-02 | S5-P04 | `4d784dc` | Min Han | PASS | `logs/2026-11-02_S5-P04-M05_phase4_fixed_distances.txt` | Robo Pico, sensor on Grove 4, 3.3 V. Raw echo lines print. Image built with S5_PHASE 4; s5_phase.h in the repo stays at 2 |
| 2026-11-02 | S5-M05 | `4d784dc` | Min Han | PARTIAL | `logs/2026-11-02_S5-P04-M05_phase4_fixed_distances.txt` | Only 3 distances, not the 4 in the test: object at roughly 10, 20 and 30 cm, reference is the iOS Measure app (about 1 to 2 cm uncertain). Reported 9, 21 and 32 cm. Echo 527 us at 9 cm (2 of 118 readings 512 us, 1 reading 572 us = 10 cm), 1169 to 1195 us at 20/21 cm (most 1189 to 1195), 1806 to 1852 us at 31/32 cm (most 1832 to 1852). 50 and 100 cm not tested. First reading after boot is status 2, 0 us |
| 2026-11-02 | S5-M07 | `4d784dc` | Min Han | PASS | `logs/2026-11-02_S5-P04-M07_phase4_sweep.txt` | Open space gives status 1 with echo about 59750 us (not about 38000 us as written in the test). 70 status 1 lines, 34251 to 59801 us, nothing is reported as status 2 in the sweep capture. status 2 appears only for the first reading after boot, see the S5-M05 log. S5_NOECHO_FAULT_N left as it is |
| 2026-11-04 | S5-M09 | `d48eb28` | Min Han | PASS | `logs/2026-11-04_S5-phase5_run1.txt` | Image built with S5_PHASE 5; s5_phase.h in the repo stays at 2. Run 1: scan 3300, 3140 and 4220 ms (seq 1 to 3), objects at about 30 cm. Target under 5 s |
| 2026-11-04 | S5-M09 | `d48eb28` | Min Han | PASS | `logs/2026-11-04_S5-phase5_run2_two_hands.txt` | Image built with S5_PHASE 5; s5_phase.h in the repo stays at 2. Run 2: scan 4220 ms (seq 9) and 3140 ms (seq 10). Target under 5 s |
| 2026-11-04 | S5-M09 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-M09_run3_9c46168.txt` | Image built with S5_PHASE 5; s5_phase.h in the repo stays at 2. Run 3: scan 3740 ms (seq 1 and 2). Target under 5 s |
| 2026-11-04 | S5-M09 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-M09-M11_run4_9c46168.txt` | Image built with S5_PHASE 5; s5_phase.h in the repo stays at 2. Run 4: scan 3200 ms (seq 1 and 2). Target under 5 s |
| 2026-11-04 | S5-M09 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-M09-M12-M13_run5_9c46168.txt` | Image built with S5_PHASE 5; s5_phase.h in the repo stays at 2. Run 5: scan 2980 ms (seq 1 to 3). Target under 5 s |
| 2026-11-04 | S5-M09 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-M09-M12_run6_9c46168.txt` | Image built with S5_PHASE 5; s5_phase.h in the repo stays at 2. Run 6: scan 3740 ms (seq 2 to 4). Target under 5 s |
| 2026-11-04 | S5-M09 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-M09-M11_run7_9c46168.txt` | Image built with S5_PHASE 5; s5_phase.h in the repo stays at 2. Run 7: scan 3520 ms (seq 1 to 3). Target under 5 s |
| 2026-11-04 | S5-M11 | `d48eb28` | Min Han | FAIL | `logs/2026-11-04_S5-phase5_run1.txt`, `logs/2026-11-04_S5-phase5_run2_two_hands.txt` | Image built with S5_PHASE 5; s5_phase.h in the repo stays at 2. My hand about 30 cm ahead and another about 30 cm to the right of the sensor. The second object was not planned around: run 1 seq 2 reported left 25 cm, right 17 cm and turned right (action 1), run 2 seq 10 identical. Run 2 seq 9 merged both into one 39 cm object. Run 1 seq 1 reported left 0 cm but still chose turn left (action 2). Fixed in `9c46168` (every cluster checked, shifts clamped before the corridor test, outer-edge planning) |
| 2026-11-04 | S5-M11 | `9c46168` | Min Han | VOID | `logs/2026-11-04_S5-M09-M11_run4_9c46168.txt`, `logs/2026-11-04_S5-M09-M12-M13_run5_9c46168.txt`, `logs/2026-11-04_S5-M09-M12_run6_9c46168.txt` | Image built with S5_PHASE 5; s5_phase.h in the repo stays at 2. Runs 4 to 6 do not count: the background scan was taken with the objects or a person in place (run 4 background 37, 41 and 39 cm at 30 to 60 deg, run 5 printed the WARNING, run 6 background 34 and 56 cm at 30 and 45 deg), and every PROFILE line shows objects 1. Test order was wrong (objects placed before the reset), not the firmware. Repeated in run 7 |
| 2026-11-04 | S5-M11 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-M09-M11_run7_9c46168.txt` | Image built with S5_PHASE 5; s5_phase.h in the repo stays at 2. Reset with nothing in front, 9 background lines with no WARNING and nothing closer than 200 cm, then one object about 30 cm ahead and a second about 30 cm to the right with a gap of at least 15 cm. Seq 2 and 3: objects 2, closest 32 cm @90 deg, left 25 cm, right 510 cm (blocked), action 2 (turn left), width 8 cm (+/-8), planned 20 cm (seq 2) and 27 cm (seq 3), scan 3520 ms. Seq 1 (before the second object was in place) showed objects 1, left 25 cm, right 49 cm, action 2 |
| 2026-11-04 | S5-M12 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-M09-M12-M13_run5_9c46168.txt` | Image built with S5_PHASE 5; s5_phase.h in the repo stays at 2. Run 5, seq 1 to 3: width 8/8/8 cm, planned 25/25/26 cm, left 24/24/24 cm, right 23/23/24 cm, action 1 every time. Largest spread 1 cm. The background was not clean at the side angles (see S5-M13 and S5-M11 VOID), so this shows repeatability only |
| 2026-11-04 | S5-M12 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-M09-M12_run6_9c46168.txt` | Image built with S5_PHASE 5; s5_phase.h in the repo stays at 2. Run 6, seq 2 to 4: width 33/33/33 cm, planned 52/53/52 cm, left 33/34/33 cm, right 41/41/41 cm, action 2 every time. Largest spread 1 cm. Background had 34 and 56 cm at 30 and 45 deg, so this shows repeatability only |
| 2026-11-04 | S5-M13 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-M09-M12-M13_run5_9c46168.txt` | Image built with S5_PHASE 5; s5_phase.h in the repo stays at 2. Run 5: a person stood ahead during the reset. Console printed `WARNING 3 background value(s) ahead were close and are ignored`, the background at 75, 90 and 105 deg was stored as 511 cm (none), and the person was still detected afterwards (closest 30 to 31 cm @90 to 105 deg, objects 1) |
| 2026-11-04 | S5-H19 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run, WSL, 46 passed 0 failed. Code unchanged between `9c46168` and `9d3a9e1`. Outlier rejected, majority "nothing" wins, n=1 and n=0 safe |
| 2026-11-04 | S5-H20 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. A reading equal to its background is not an obstacle |
| 2026-11-04 | S5-H21 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. Box in front of a wall: cluster 5..7, edges found against the wall |
| 2026-11-04 | S5-H22 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. a: 4 cm closer extends the cluster, 2 cm closer does not. b: 4 cm closer alone cannot start one |
| 2026-11-04 | S5-H23 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. a: 53 cm stays in the cluster. b: 57 cm is out |
| 2026-11-04 | S5-H24 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. a: 3 close background values ahead forgotten. b: something 30 cm dead ahead at start-up is still an obstacle |
| 2026-11-04 | S5-H25 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. a: width about 8 cm, planned about 25 cm. b: with plan_width_cm 0 the plan is never narrower than measured. c: 20 cm floor gives shift 21 cm against about 15 without. d: a wide obstacle is never planned narrower than measured |
| 2026-11-04 | S5-H26 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. Uncertainty about +/-7 cm at 30 cm |
| 2026-11-04 | S5-H27 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. Simulation 0..180 deg: 390 cases, 0 unsafe or missed, worst clearance 3 cm |
| 2026-11-04 | S5-H28 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. Simulation 60..120 deg: 390 cases, 0 unsafe or missed, worst clearance 3 cm |
| 2026-11-04 | S5-H29 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. Simulation 45..135 deg: 390 cases, 0 unsafe or missed, worst clearance 3 cm |
| 2026-11-04 | S5-H30 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. Simulation 30..150 deg: 390 cases, 0 unsafe or missed, worst clearance 3 cm |
| 2026-11-04 | S5-H31 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. Hardware regression of the 2026-11-04 run: centre box plus box on the right, 3 clusters, left 26 cm, right 510 cm, turn left (action 2) |
| 2026-11-04 | S5-H32 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. Boxes on both sides: left 510 cm, right 48 cm, REVERSE_RETRY (action 3) |
| 2026-11-04 | S5-H33 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. One wide obstacle 38 cm: a single cluster, left 33 cm, right 48 cm, action 2 |
| 2026-11-04 | S5-H34 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. a: from index 10 the next is 7. b: then 5. c: then 3. d: index 1 is outside the servo window, nothing left to measure |
| 2026-11-04 | S5-H35 | `9c46168` | Min Han | PASS | `logs/2026-11-04_S5-H01-H35.txt` | PC run. Two boxes, 30..150 deg: 980 cases, 0 unsafe or missed, 872 bypass chosen, 108 reverse-retry or stop |
