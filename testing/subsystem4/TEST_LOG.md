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


## Phase 3 - Calibration and Tilt Direction

**Latest run: 2026-10-28_180316. Functional result: PASS.**

Scope: level calibration, pitch/roll direction and startup movement rejection/recovery on the bench. PASS applies to these functional checks; numerical measurement accuracy and other features are outside this record's scope.

### Test cases

| ID | What is tested | Method | Expected result |
|---|---|---|---|
| S4-U05 | Driver, processing/math and deadline regression checks | Automated portable C assertions | Assertions pass; synthetic checks do not establish physical accuracy |
| S4-H02 | Level calibration, pitch/roll direction and startup movement rejection/recovery | Guided bench run with saved console output and operator-confirmed steady poses | Calibration completes while level; front-up/down and left/right signs match; calibration waits during startup movement and completes after resting |

### Latest results

| Date | Test ID | Commit | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-28 | S4-U05 | `31a6b57` | Bryan | PASS | [Host](logs/2026-10-28_180316_phase3_host.txt) | Portable assertions executed successfully. |
| 2026-10-28 | S4-H02 | `31a6b57` | Bryan | PASS (functional scope) | [Console](logs/2026-10-28_180316_phase3_sensor.txt); [Reviewed summary](logs/2026-10-28_180316_phase3_summary.json); [Build](logs/2026-10-28_180316_phase3_build.txt) | Level calibration, both pitch/roll directions and startup movement rejection/recovery passed in the observed bench run. |

### Verified functional checks

| Check | Result | Observation |
|---|---|---|
| Level calibration | PASS | Calibration completed with the sensor level. |
| Pitch direction | PASS | Front-up mean +17.237 degrees; front-down mean -20.909 degrees. Both poses confirmed steady. |
| Roll direction | PASS | Left-side-up mean +23.206 degrees; right-side-up mean -19.256 degrees. Both poses confirmed steady. |
| Startup movement rejection | PASS | All 15 captured moving-stage rows remained uncalibrated. |
| Calibration after movement stops | PASS | Calibration completed after resting; 23 calibrated rows captured. |

Firmware checkpoint: `31a6b57ff01590cfad904c9e6d387c997ab0412c`. All 148 captured paired rows were fresh with read return code zero and missed counter zero. This does not establish worst-case execution time or exact sampling jitter.




## Phase 4 test cases

| ID | What is tested | Method | Expected result |
|---|---|---|---|
| S4-U06 | Portable terrain FSM and regression checks | Automated C assertions | Ordered/debounced traversal; fresh-data gating; impact rearm; timeout/wrap and fault latch assertions pass |
| S4-H03 | Calibration, flat baseline, ordered humps and incomplete traversal rejection | Guided sensor bench movements | Level calibration; flat baseline without hump; three ordered single-hump completions; incomplete and downhill-only movements do not produce a hump |
| S4-H04 | Impact event and rearm functional observation | Optional two separate padded-support bumps | One captured impact event per confirmed single bump; no impact during the rest interval; no physical latency claim |
| S4-H05 | Sensor fault latch and restart recovery | Optional connector disconnect/reconnect followed by Pico restart | FAULT on sensor loss; reconnect alone does not clear it; level calibration and LEVEL after restart |





### Phase 4 run 2026-11-01_181511_666687

| Date | Test ID | Commit | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-01 | S4-U06 | `a522381` | Bryan | PASS | [Host](logs/2026-11-01_181511_666687_phase4_host.txt) | Portable terrain behavior assertions plus driver, processing and timer regression checks. |
| 2026-11-01 | S4-H03 | `a522381` | Bryan | PASS | [Console](logs/2026-11-01_181511_666687_phase4_sensor.txt); [Summary](logs/2026-11-01_181511_666687_phase4_summary.json) | Sensor bench functional scope: flat=PASS; three humps=PASS,PASS,PASS; incomplete=PASS; downhill-only=PASS. No track detection-rate, accuracy or timing claim. |
| 2026-11-01 | S4-H04 | `a522381` | Bryan | NOT RUN | [Console](logs/2026-11-01_181511_666687_phase4_sensor.txt); [Summary](logs/2026-11-01_181511_666687_phase4_summary.json) | Optional impact bench check skipped by operator; host rearm assertions are separate. |
| 2026-11-01 | S4-H05 | `a522381` | Bryan | NOT RUN | [Console](logs/2026-11-01_181511_666687_phase4_sensor.txt); [Summary](logs/2026-11-01_181511_666687_phase4_summary.json) | Optional sensor-disconnection check skipped by operator. |


## Phase 5 — hump-height estimation

The main physical demonstrations are detecting a complete hump, estimating its height using IMU pitch and real wheel distance, and keeping the highest valid estimate. Motor driving and line sensing belong to the other subsystems; they provide the motion and inputs for the integrated S4 test.

### Supporting software checks

These host checks protect the estimator from invalid inputs. Keep their executable tests; they are not separate live demonstrations of hump-height accuracy.

| ID | What is tested | Method | Expected result |
|---|---|---|---|
| S4-U07 | Phase 5 height/odometry guards | Synthetic C reference profile and rejection assertions | Software assertions pass; no physical accuracy claim |
| S4-U08 | Timebase-check logic | Synthetic clock boundaries and wrap | Software assertions pass; not calibrated hardware time |
| S4-U09 | Odometry callback contract | Production callback code with host API stubs | Unbound/valid/refused binding cases pass |
| S4-U10 | Live S2 input decoder/guards | Synthetic IF-17 words through actual reader | Freshness, reverse, gap, duplicate, epoch and wrap cases pass |
| S4-U11 | S4/S5 glue and fault ownership | Actual glue with host RTOS/S2 stubs | Input callback, highest snapshot, correct fault-bit OR and no clearing; no SMP claim |
| S4-U12 | Powered test stop policy | Synthetic distance, count, timeout and wrap cases | Software stop/latched decision assertions pass; not stopping-distance proof |
| S4-H11 | Flat forward distance readiness | One independently measured flat segment with live encoders; record hand-pushed or motor-powered mode | Distance error <=5%, level, no false hump; not full S2-M06 |
| S4-H10 | Actual hump-height accuracy | Three measured school-hump forward car traversals; record hand-pushed or motor-powered mode | Valid height on every proper traversal and each error <=10%; highest consistent within each run |

### Main physical results — powered straight hump trials, 2026-11-06

Tester: Bryan (`xcher21`). Profile: HEIGHT_ONLY; commanded speed 50 mm/s; S2 encoders + S4 IMU + Safety Monitor. No line/barcode/network/obstacle task. Reference supplied by operator: 23 mm. Configured wheelbase: 110 mm; independent physical confirmation is not recorded here.

Tested build: `WT-e719d81481cc`, working snapshot `e719d81481cc06f953c69a5a0b04188ca2dd6d0f210d7394ab353f340d5fdeb7`, based on commit `ae2bfc2156178683bcd3e84fcec71a4bfe50802b`. These were working-tree tests; the base commit alone does not identify all test-image overrides. See [tested build manifest](logs/2026-11-06_tested_build.json) and [summary](logs/2026-11-06_three_trials_summary.json). Do not substitute a later documentation/code commit as the tested revision.

### Main physical test cases

| ID | Purpose | Procedure | Acceptance | Result |
|---|---|---|---|---|
| S4-H10 | Actual hump-height accuracy | Three independent powered traversals from flat calibration, across measured hump, onto flat ground | Every valid traversal has absolute error <=10% | **FAIL: 2/3 within target** |
| S4-H12 | Powered hump detection and completion | Same captures; check LEVEL → CLIMBING → DESCENDING → LEVEL, published height/highest and automatic HUMP stop request | All three captures show a completion, estimate and HUMP stop request | **PASS: 3/3** |
| S4-H13 | Highest valid hump in one boot | Traverse at least two independently measured humps of different heights without resetting | Highest stored equals the largest valid estimate and does not decrease on the smaller hump | **NOT RUN** |

S4-H10 existed before these runs. S4-H12 summarises functional observations from these same captures. Highest value was observed within each boot; maximum selection across different humps in one boot remains NOT RUN.

| Trial | Completion (SGT) | Reference mm | Estimate mm | Absolute error | Functional | Accuracy <=10% | Evidence |
|---|---|---|---|---|---|---|---|
| 1 | 23:22:58 | 23 | 24 | 4.3% | PASS | PASS | [Capture](logs/2026-11-06_50mmps_trial1_console.txt) |
| 2 | 23:28:35 | 23 | 17 | 26.1% | PASS | FAIL | [Capture](logs/2026-11-06_50mmps_trial2_console.txt) |
| 3 | 23:32:21 | 23 | 21 | 8.7% | PASS | PASS | [Capture](logs/2026-11-06_50mmps_trial3_console.txt) |

All three completion messages disclose two interpolated samples and a longest interval of 40 ms. Trial 1 includes raw S4 telemetry; trials 2/3 are pasted console displays. Three estimates are 24/17/21 mm (range 7 mm, mean 20.67 mm). The failed 17-mm accuracy trial remains part of this set. Results do not establish full robot integration, independent encoder accuracy, complete WCET or compliance across terrain types.

### Evidence retained after cleanup

The current evidence folder keeps the three latest trial captures, their summary and tested-build identity, plus the evidence supporting Phases 1–4. Older development archive files and duplicate test-fixture code snapshots were removed at Bryan's request. This cleanup does not change the latest accuracy failure: the 17-mm trial remains recorded above.

### Next integrated test — planned, NOT RUN

- Confirm the physical axle-to-caster distance used by the configured wheelbase.
- Run with the agreed integrated image and a continuous line; record its actual code/build identity. Today's HEIGHT_ONLY image has no line task.
- Check S3 line readings and S2 forward motion while S4 reports terrain and height. Coordinate shared integration IDs with those subsystem owners.
- Repeat the measured hump crossings; record every estimate and its error. The current three-run accuracy result is still FAIL until a new series demonstrates the requirement.
- Test the highest value across different humps without rebooting (S4-H13), and confirm S5 receives S4's published highest value.

The straight-driving stop is a test-harness convenience, not a new S4 responsibility. S4 reports sensor faults; the Safety Monitor owns the response and fault clearing. Do not claim barcode/network/obstacle or full mission integration from this isolated height test.
