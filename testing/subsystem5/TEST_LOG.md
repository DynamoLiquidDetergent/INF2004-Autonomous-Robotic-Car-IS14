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