# Subsystem 4 - Test Log

Subsystem: Physics Watchdog - IMU-Based Motion and Terrain Monitoring

Student ID: 2500767

Owner: Bryan (`xcher21`)

Source directory: `app_program/subsystem4/`

Testing directory: `testing/subsystem4/`

Follow `testing/README.md`. Keep previous run records and use the commit of the code actually tested.

## Phase 1 status

PASS for the portable deadline checks and the Pico task startup, priority and ten-minute observation checks.

Tested code commit: `b449c60633a348ef0fe2fd8adfa2f46b470e8064`.

The board result applies to the task foundation. IMU measurements, terrain detection and hump-height estimation were not tested in Phase 1.

## Test cases
| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | message buffer max line = S4_SUMMARY_MAX | IF-03 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | IMU task foundation | Week 6 S4 task allocation | Host and board | Run deadline assertions; query task priority; capture ten minutes of console output. | Deadline assertions pass; task exists at priority 18; heartbeat counter advances; missed releases and resets are recorded. |

The configured period is 20 ms. The board capture checks task operation; it does not measure precise release jitter or worst-case execution time. No S4 LED light-up check is required for this case. Other subsystems' example checks appear in the full firmware console output.

## Results

Append a new row for each run. Preserve earlier blocked and failed attempts.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-27 | S4-U01 | `b449c60` | Bryan | PASS | `logs/2026-10-27_213347_phase1_host.txt` | Host portion only: deadline, overrun and timer wrap assertions. In-process TinyCC; |
| 2026-10-27 | S4-U02 | `b449c60` | Bryan | PASS | [Console](logs/2026-10-27_223645_phase1_board.txt); [Summary](logs/2026-10-27_223645_phase1_board_summary_public.json); [Build](logs/2026-10-27_223645_phase1_build_public.txt) | Board portion: priority 18 confirmed; 600.062 seconds; 600 heartbeat lines; cycles increased from 100 to 30050; missed=0 throughout; no counter reset or test failure observed. |


## Phase 1 board evidence

| Item | Recorded value |
|---|---|
| Date | 2026-10-27, Asia/Singapore |
| Firmware source commit | `b449c60633a348ef0fe2fd8adfa2f46b470e8064` |
| Firmware build | Completed; exit code 0 |
| Build profile | `SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=none MQTT=0 CRASH_REPORT=0 TEST=1` |
| Flash | Copied to the RP2040 BOOTSEL drive `D:\` |
| Console | COM4, 115200 baud, 8N1 |
| S4 startup test | `[TEST] S4-U03 PASS` |
| Reported task priority | 18 |
| Observation duration | 600.062 seconds |
| Heartbeat lines captured | 600 |
| First captured counter | cycles=100, missed=0 |
| Last captured counter | cycles=30050, missed=0 |
| Missed-release counter | 0 in every captured heartbeat |
| Counter resets observed | None |
| Test failure lines observed | None |
| Firmware SHA-256 | `c28c83ae86650beb9e1951c2ce4435b4fe1c991d0f3f8f82466b7be6e5e9e871` |

## Pending validation

- Precise task release jitter and worst-case execution time.
- Stack headroom and target resource measurements.
- Sensor readings and calibration after the motion sensor is available.
- Terrain, impact, hump-height and integration checks in later phases.

The template's placeholder result was not a completed test and is not included in the Results table.

## Evidence copies

The successful board summary and build-log links use explicitly marked redacted copies. Only absolute local path information was removed. The original logs, timestamps, measurements and earlier attempts remain unchanged. The successful board observation was one continuous 600.062-second capture.

## Phase 2 test cases

| ID | What is tested | Method | Expected result |
|---|---|---|---|
| S4-U04 | Motion sensor driver configuration, decoding and error handling | Automated portable C mock test | Driver and deadline assertions pass |
| S4-H01 | Sensor communication, six resting positions and disconnected-sensor response | Guided bench run with timestamped console capture | Fresh successful readings; opposite gravity directions; missing sensor returns errors without stopping output |

### Phase 2 run 2026-10-28_130430

| Date | Test ID | Commit | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-28 | S4-U04 | `4a779d0` | Bryan | PASS | [Log](logs/2026-10-28_130430_phase2_host.txt) | Portable mock checks only; physical sensor behaviour is recorded separately. |
| 2026-10-28 | S4-H01 | `4a779d0` | Bryan | PASS | [Console](logs/2026-10-28_130430_phase2_sensor.txt); [Summary](logs/2026-10-28_130430_phase2_summary.json); [Build](logs/2026-10-28_130430_phase2_build.txt) | Communication: PASS; six positions: PASS (functional check); disconnected response: PASS; reconnection: PASS. Functional bring-up only; no calibrated accuracy, timing bound or magnetic-field accuracy is claimed. |
