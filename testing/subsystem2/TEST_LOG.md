# Subsystem 2 — test log

**Subsystem:** Motion Control (motors, wheel encoders, PID)  ·  **Student ID:** 2500461  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

### Automatic, on the Pico (`make TEST=1`)

None of these turns the motors: every command they send has speed 0 or is blocked.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | sample values stay within 0..100 (template example) | | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | free slots and free blocks within pool size (template example) | IF-02 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | motion command pack/unpack at the field limits, 27 bits used | IF-10, minimum-bit rule | auto | `subsystem2_test.c` | `[TEST] S2-U03 PASS` |
| S2-U04 | stale epochs detected, including the 15 → 0 wrap | IF-10 | auto | `subsystem2_test.c` | `[TEST] S2-U04 PASS` |
| S2-U05 | motion_task runs every 20 ms | NFR9 | auto | `subsystem2_test.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | cycle WCET < 1 ms and release jitter < 2 ms (also prints the measured values) | NFR1, NFR9 | auto | `subsystem2_test.c`; copy the printed `S2 timing:` line into Notes | `[TEST] S2-U06 PASS` |
| S2-U07 | STOP accepted: epoch reported, state IDLE, PWM 0 | IF-10, IF-16 | auto | `subsystem2_test.c` | `[TEST] S2-U07 PASS` |
| S2-U08 | command with an older epoch is discarded | IF-10 | auto | `subsystem2_test.c` | `[TEST] S2-U08 PASS` |
| S2-U09 | no command for 100 ms → state TIMEOUT, motors stopped | IF-10 | auto | `subsystem2_test.c` | `[TEST] S2-U09 PASS` |
| S2-U10 | inhibit: commands discarded, PWM 0, released only by a newer epoch (skipped once subsystem 5 binds its own flag) | IF-11, FR46 | auto | `subsystem2_test.c` | `[TEST] S2-U10 PASS` or `SKIP` |
| S2-U11 | shared GPIO hook refuses encoder pins and bad pin numbers | GPIO IRQ (TEAM.md §3) | auto | `subsystem2_test.c` | `[TEST] S2-U11 PASS` |

### Automatic, on a PC (no Pico)

Run from the repository root: `sh testing/subsystem2/host/run_host_tests.sh`

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-H01 | every mode/speed/epoch/param round-trips through the command word | IF-10 | auto (PC) | `host/test_motion_math.c` | PASS |
| S2-H02 | command boundaries: −511, 1023 mm, −180°, clamp 900 → 511 | IF-10 | auto (PC) | as above | PASS |
| S2-H03 | status word round trip, 28 bits used | IF-16 | auto (PC) | as above | PASS |
| S2-H04 | odometry word and wrap-safe difference | IF-17 | auto (PC) | as above | PASS |
| S2-H05 | epoch staleness with wrap-around | IF-10 | auto (PC) | as above | PASS |
| S2-H06 | distance per pulse = 10210 µm for 65 mm / 20 pulses | A2 | auto (PC) | as above | PASS |
| S2-H07 | speed from edge period: ±200 mm/s, decay, timeout, clamp | FR13 | auto (PC) | as above | PASS |
| S2-H08 | feed-forward duty | FR14 | auto (PC) | as above | PASS |
| S2-H09 | PI: never reverses the wheel, anti-windup, target 0 resets | FR14 | auto (PC) | as above | PASS |
| S2-H10 | setpoint ramp ≤ 25 mm/s per cycle (soft start) | supply dip risk | auto (PC) | as above | PASS |
| S2-H11 | 90° turn arc = 94.26 mm per wheel | FR11, NFR11 | auto (PC) | as above | PASS |
| S2-H12 | straight-line correction sign and limit | FR15, NFR10 | auto (PC) | as above | PASS |

### Manual, on the bench

Do S2-M01 and S2-M02 first: the results set the values at the top of `subsystem2.h`.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-M01 | encoder type: single channel or A/B quadrature | A2 | manual | Turn each wheel slowly by hand while watching GP14/GP15 and GP4/GP5 (scope, or a short print loop). | Note which. If quadrature, set `S2_ENC_QUADRATURE 1`. |
| S2-M02 | pulses per revolution and wheel diameter | A2 | manual | Tape mark on the wheel, turn exactly 10 turns by hand, read `edges` (or count). Measure the wheel diameter. | Update `S2_ENC_PULSES_PER_REV`, `S2_WHEEL_DIAMETER_MM`. |
| S2-M03 | direction and dead zone, wheels off the ground | FR11 | manual | `make TEST=1` with `S2_TEST_DRIVE 1`, robot on a box. | Both wheels turn forward. If one runs backwards, set its `S2_MOTOR_x_INVERT`. |
| S2-M04 | PID tuning: step 0 → 200 mm/s | FR14, NFR9 | manual | Build with `S2_LOG_EVERY_CYCLES 1` (top of `motion_control.c`), log the CSV, plot setpoint vs speed. | Settles within ±10% in < 1 s, no oscillation. Record the final gains. |
| S2-M05 | wheels-up drive at 200 mm/s for 2 s | FR11–FR14 | manual | `S2_TEST_DRIVE 1`; read the printed speeds. | Both speeds 180–220 mm/s. |
| S2-M06 | distance accuracy: 1 m at 3 speeds, 5 runs each | A2, NFR13 | manual | Send DRIVE_DIST 1000 mm at 100, 200 and 300 mm/s; compare with a tape measure. | Within ±5%. |
| S2-M07 | straight line over 2 m | NFR10 | manual | DRIVE with steering 0 along a taped line; measure the sideways offset. | ≤ 100 mm. |
| S2-M08 | turns: TURN +90, −90, +180, 10 each | NFR11 | manual | Mark the start heading; measure with a protractor. | Within ±5°. |
| S2-M09 | stop time from 200 mm/s | NFR12 | manual | Send STOP; log the time until speed reads 0, or film it. | ≤ 200 ms. |
| S2-M10 | stall detection | FR18, IF-12 | manual | Hold one wheel while driving. | State STALLED within ~300 ms, motors off, fault bit 0 set. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U03 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-29 | S2-H01 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_host.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-H02 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_host.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-H03 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_host.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-H04 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_host.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-H05 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_host.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-H06 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_host.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-H07 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_host.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-H08 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_host.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-H09 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_host.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-H10 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_host.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-H11 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_host.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-H12 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_host.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-U01 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1039_unit.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-U02 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1039_unit.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-U03 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1039_unit.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-U04 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1039_unit.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-U05 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1039_unit.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-U06 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1039_unit.txt` | WCET 220 us, jitter 323 us (tested with uncommitted changes) |
| 2026-10-29 | S2-U07 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1039_unit.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-U08 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1039_unit.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-U09 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1039_unit.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-U10 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1039_unit.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-U11 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1039_unit.txt` | (tested with uncommitted changes) |
| 2026-10-29 | S2-M01 | `d900e4a` | Aaron | PASS | `multimeter` | encoder type: quadrature (A+B) (tested with uncommitted changes) |
| 2026-10-29 | S2-M02 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1041_pulses.txt` | 635 pulses/turn (L 634.2, R 635.4), wheel 65 mm (tested with uncommitted changes) |
| 2026-10-29 | S2-M03 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1045_drive.txt` | both wheels forward (tested with uncommitted changes) |
| 2026-10-29 | S2-M05 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1045_drive.txt` | L 205, R 202 mm/s (target 180-220) (tested with uncommitted changes) |
| 2026-10-29 | S2-M04 | `d900e4a` | Aaron | FAIL | `logs/2026-10-29_1046_tune.csv` | L settle 1320 ms, overshoot 62%; R settle 1300 ms, overshoot 52%; KP 800 KI 60 FF 1250 (tested with uncommitted changes) |
| 2026-10-29 | S2-M04 | `d900e4a` | Aaron | FAIL | `logs/2026-10-29_1051_tune.csv` | L settle 160 ms, overshoot 5%; R settle 300 ms, overshoot 1%; KP 800 KI 60 FF 560 (tested with uncommitted changes) |
| 2026-10-29 | S2-M04 | `d900e4a` | Aaron | FAIL | `logs/2026-10-29_1052_tune.csv` | L settle 160 ms, overshoot 5%; R settle 400 ms, overshoot 1%; KP 800 KI 60 FF 560 (tested with uncommitted changes) |
| 2026-10-29 | S2-M04 | `d900e4a` | Aaron | FAIL | `logs/2026-10-29_1056_tune.csv` | L settle 160 ms, overshoot 5%; R settle 340 ms, overshoot 0%; KP 800 KI 60 FF 560 (tested with uncommitted changes) |
| 2026-10-29 | S2-M10 | `d900e4a` | Aaron | FAIL | `logs/2026-10-29_1057_stall.txt` | state STALLED never reached (tested with uncommitted changes) |
| 2026-10-29 | S2-M03 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1058_drive.txt` | both wheels forward (tested with uncommitted changes) |
| 2026-10-29 | S2-M05 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1058_drive.txt` | L 204, R 201 mm/s (target 180-220) (tested with uncommitted changes) |
| 2026-10-29 | S2-M08 | `d900e4a` | Aaron | PASS | `logs/2026-10-29_1104_turns.txt` | wheels up: direction only (tested with uncommitted changes) |
