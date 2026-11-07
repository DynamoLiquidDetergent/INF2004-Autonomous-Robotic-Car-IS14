# Subsystem 4 — test log

**Subsystem:** Physics Watchdog — IMU-Based Motion and Terrain Monitoring · **Student ID:** 2500767 · **Owner:** Bryan (`xcher21`) · **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Test IDs are retained. Host tests check portable software logic; manual/board tests establish only their recorded hardware scope.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | message buffer max line = S4_SUMMARY_MAX | IF-03 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | IMU task foundation | Week 6 S4 task allocation | Host and board | Run deadline assertions; query task priority; capture ten minutes of console output. | Deadline assertions pass; task exists at priority 18; heartbeat counter advances; missed releases and resets are recorded. |
| S4-U04 | Motion sensor driver configuration, decoding and error handling | See original phase scope | Automated portable C mock test | Automated portable C mock test | Driver and deadline assertions pass |
| S4-H01 | Sensor communication, six resting positions and disconnected-sensor response | See original phase scope | Guided bench run with timestamped console capture | Guided bench run with timestamped console capture | Fresh successful readings; opposite gravity directions; missing sensor returns errors without stopping output |
| S4-U05 | Driver, processing/math and deadline regression checks | See original phase scope | Automated portable C assertions | Automated portable C assertions | Assertions pass; synthetic checks do not establish physical accuracy |
| S4-H02 | Level calibration, pitch/roll direction and startup movement rejection/recovery | See original phase scope | Guided bench run with saved console output and operator-confirmed steady poses | Guided bench run with saved console output and operator-confirmed steady poses | Calibration completes while level; front-up/down and left/right signs match; calibration waits during startup movement and completes after resting |
| S4-U06 | Portable terrain FSM and regression checks | See original phase scope | Automated C assertions | Automated C assertions | Ordered/debounced traversal; fresh-data gating; impact rearm; timeout/wrap and fault latch assertions pass |
| S4-H03 | Calibration, flat baseline, ordered humps and incomplete traversal rejection | See original phase scope | Guided sensor bench movements | Guided sensor bench movements | Level calibration; flat baseline without hump; three ordered single-hump completions; incomplete and downhill-only movements do not produce a hump |
| S4-H04 | Impact event and rearm functional observation | See original phase scope | Optional two separate padded-support bumps | Optional two separate padded-support bumps | One captured impact event per confirmed single bump; no impact during the rest interval; no physical latency claim |
| S4-H05 | Sensor fault latch and restart recovery | See original phase scope | Optional connector disconnect/reconnect followed by Pico restart | Optional connector disconnect/reconnect followed by Pico restart | FAULT on sensor loss; reconnect alone does not clear it; level calibration and LEVEL after restart |
| S4-U07 | Phase 5 height/odometry guards | See original phase scope | Synthetic C reference profile and rejection assertions | Synthetic C reference profile and rejection assertions | Software assertions pass; no physical accuracy claim |
| S4-U08 | Timebase-check logic | See original phase scope | Synthetic clock boundaries and wrap | Synthetic clock boundaries and wrap | Software assertions pass; not calibrated hardware time |
| S4-U09 | Odometry callback contract | See original phase scope | Production callback code with host API stubs | Production callback code with host API stubs | Unbound/valid/refused binding cases pass |
| S4-U10 | Live S2 input decoder/guards | See original phase scope | Synthetic IF-17 words through actual reader | Synthetic IF-17 words through actual reader | Freshness, reverse, gap, duplicate, epoch and wrap cases pass |
| S4-U11 | S4/S5 glue and fault ownership | See original phase scope | Actual glue with host RTOS/S2 stubs | Actual glue with host RTOS/S2 stubs | Input callback, highest snapshot, correct fault-bit OR and no clearing; no SMP claim |
| S4-U12 | Powered test stop policy | See original phase scope | Synthetic distance, count, timeout and wrap cases | Synthetic distance, count, timeout and wrap cases | Software stop/latched decision assertions pass; not stopping-distance proof |
| S4-H11 | Flat forward distance readiness | See original phase scope | One independently measured flat segment with live encoders; record hand-pushed or motor-powered mode | One independently measured flat segment with live encoders; record hand-pushed or motor-powered mode | Distance error <=5%, level, no false hump; not full S2-M06 |
| S4-H10 | Actual hump-height accuracy | See original phase scope | Three measured school-hump forward car traversals; record hand-pushed or motor-powered mode | Three measured school-hump forward car traversals; record hand-pushed or motor-powered mode | Valid height on every proper traversal and each error <=10%; highest consistent within each run |
| S4-H12 | Powered hump detection and completion | Phase 5 | manual | Same captures; check LEVEL → CLIMBING → DESCENDING → LEVEL, published height/highest and automatic HUMP stop request | All three captures show a completion, estimate and HUMP stop request |
| S4-H13 | Highest valid hump in one boot | Phase 5 | manual | Traverse at least two independently measured humps of different heights without resetting | Highest stored equals the largest valid estimate and does not decrease on the smaller hump |
| S4-H14 | Fixed display-correction experiment on a measured right-wheel hump | S4 height display; not uncorrected firmware accuracy | manual | Three powered traversals, 120 mm wheel spacing, 23 mm reference, fixed 5 mm display correction; preserve original captures | Each displayed estimate within ±10%; one recorded hump per traversal |

*Method:* **auto / host** = software assertions on the indicated platform; **manual / board** = physical observations with saved evidence. Synthetic model checks are not physical accuracy results.

## Results

Earlier results are restored without changing their PASS/FAIL/NOT RUN outcomes. Add a new row for every subsequent run.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-27 | S4-U01 | `b449c60` | Bryan | PASS | `logs/2026-10-27_213347_phase1_host.txt` | Host portion only: deadline, overrun and timer wrap assertions. In-process TinyCC; historical PASS retained, but the originally cited host capture is not present in the available backup. |
| 2026-10-27 | S4-U02 | `b449c60` | Bryan | PASS | [Console](logs/2026-10-27_223645_phase1_board.txt); [Summary](logs/2026-10-27_223645_phase1_board_summary.json); [Build](logs/2026-10-27_223645_phase1_build.txt) | Board portion: priority 18 confirmed; 600.062 seconds; 600 heartbeat lines; cycles increased from 100 to 30050; missed=0 throughout; no counter reset or test failure observed. |
| 2026-10-28 | S4-U04 | `4a779d0` | Bryan | PASS | [Log](logs/2026-10-28_130430_phase2_host.txt) | Portable mock checks only; physical sensor behaviour is recorded separately. |
| 2026-10-28 | S4-H01 | `4a779d0` | Bryan | PASS | [Console](logs/2026-10-28_130430_phase2_sensor.txt); [Summary](logs/2026-10-28_130430_phase2_summary.json); [Build](logs/2026-10-28_130430_phase2_build.txt) | Communication: PASS; six positions: PASS (functional check); disconnected response: PASS; reconnection: PASS. Functional bring-up only; no calibrated accuracy, timing bound or magnetic-field accuracy is claimed. |
| 2026-10-28 | S4-U05 | `31a6b57` | Bryan | PASS | [Host](logs/2026-10-28_180316_phase3_host.txt) | Portable assertions executed successfully. |
| 2026-10-28 | S4-H02 | `31a6b57` | Bryan | PASS (functional scope) | [Console](logs/2026-10-28_180316_phase3_sensor.txt); [Reviewed summary](logs/2026-10-28_180316_phase3_summary.json); [Build](logs/2026-10-28_180316_phase3_build.txt) | Level calibration, both pitch/roll directions and startup movement rejection/recovery passed in the observed bench run. |
| 2026-11-01 | S4-U06 | `a522381` | Bryan | PASS | [Host](logs/2026-11-01_181511_666687_phase4_host.txt) | Portable terrain behavior assertions plus driver, processing and timer regression checks. |
| 2026-11-01 | S4-H03 | `a522381` | Bryan | PASS | [Console](logs/2026-11-01_181511_666687_phase4_sensor.txt); [Summary](logs/2026-11-01_181511_666687_phase4_summary.json) | Sensor bench functional scope: flat=PASS; three humps=PASS,PASS,PASS; incomplete=PASS; downhill-only=PASS. No track detection-rate, accuracy or timing claim. |
| 2026-11-01 | S4-H04 | `a522381` | Bryan | NOT RUN | [Console](logs/2026-11-01_181511_666687_phase4_sensor.txt); [Summary](logs/2026-11-01_181511_666687_phase4_summary.json) | Optional impact bench check skipped by operator; host rearm assertions are separate. |
| 2026-11-01 | S4-H05 | `a522381` | Bryan | NOT RUN | [Console](logs/2026-11-01_181511_666687_phase4_sensor.txt); [Summary](logs/2026-11-01_181511_666687_phase4_summary.json) | Optional sensor-disconnection check skipped by operator. |
| 2026-11-06 | S4-H10 | `WT-e719d81481cc` (base `ae2bfc2`) | Bryan | PASS | [Capture](logs/2026-11-06_50mmps_trial1_console.txt) | Phase 5 straight-driving trial 1, 23:22:58 SGT: reference 23 mm; estimate 24 mm; error 4.3%. Original result retained. |
| 2026-11-06 | S4-H10 | `WT-e719d81481cc` (base `ae2bfc2`) | Bryan | FAIL | [Capture](logs/2026-11-06_50mmps_trial2_console.txt) | Phase 5 straight-driving trial 2, 23:28:35 SGT: reference 23 mm; estimate 17 mm; error 26.1%. Original result retained. |
| 2026-11-06 | S4-H10 | `WT-e719d81481cc` (base `ae2bfc2`) | Bryan | PASS | [Capture](logs/2026-11-06_50mmps_trial3_console.txt) | Phase 5 straight-driving trial 3, 23:32:21 SGT: reference 23 mm; estimate 21 mm; error 8.7%. Original result retained. |
| 2026-11-06 | S4-H12 | `WT-e719d81481cc` (base `ae2bfc2`) | Bryan | PASS | [Summary](logs/2026-11-06_three_trials_summary.json) | Phase 5 functional completion 3/3; HEIGHT_ONLY profile issued its automatic hump stop. Does not override the S4-H10 accuracy failure. |
| 2026-11-06 | S4-H13 | `WT-e719d81481cc` (base `ae2bfc2`) | Bryan | NOT RUN | [Build](logs/2026-11-06_tested_build.json) | Different-height humps in one boot were not tested. |
| 2026-11-07 | S4-H14 | `WT-609dab8d2247` (base `2d287d7`) | Bryan | PASS (display experiment) | [Capture](logs/2026-11-07_223022_054533_right_wheel_hump_trial1.txt) | Right-wheel trial 1, 22:30:40 SGT: displayed height 24 mm; reference 23 mm; error 4.3%; fixed 5 mm display correction; exactly one hump. |
| 2026-11-07 | S4-H14 | `WT-609dab8d2247` (base `2d287d7`) | Bryan | PASS (display experiment) | [Capture](logs/2026-11-07_223135_967778_right_wheel_hump_trial2.txt) | Right-wheel trial 2, 22:31:54 SGT: displayed height 24 mm; reference 23 mm; error 4.3%; fixed 5 mm display correction; exactly one hump. |
| 2026-11-07 | S4-H14 | `WT-609dab8d2247` (base `2d287d7`) | Bryan | PASS (display experiment) | [Capture](logs/2026-11-07_223235_721306_right_wheel_hump_trial3.txt) | Right-wheel trial 3, 22:32:57 SGT: displayed height 23 mm; reference 23 mm; error 0.0%; fixed 5 mm display correction; exactly one hump. |
| 2026-11-07 | S4-H12 | `WT-609dab8d2247` (base `2d287d7`) | Bryan | PASS (three-run scope) | [Trial 1](logs/2026-11-07_223022_054533_right_wheel_hump_trial1.txt); [Trial 2](logs/2026-11-07_223135_967778_right_wheel_hump_trial2.txt); [Trial 3](logs/2026-11-07_223235_721306_right_wheel_hump_trial3.txt) | Right-wheel mode: all three runs completed LEVEL → CLIMBING → DESCENDING → LEVEL, retained heights, no reported read error, missed release, reverse motion, line loss or Safety fault. This profile continues driving after a hump. |

## Latest displayed height summary

| Trial | Displayed height (mm) | Error against 23 mm | Within ±10% |
|---|---|---|---|
| 1 | 24 | 4.3% | PASS (display experiment) |
| 2 | 24 | 4.3% | PASS (display experiment) |
| 3 | 23 | 0.0% | PASS (display experiment) |

Method: fixed 5 mm console-only correction. Firmware-stored heights are not changed. This supports the three-run S4-H14 display experiment on this 23 mm hump; it does not establish uncorrected firmware accuracy or calibration across other heights. S4-H10's previous raw accuracy failure remains part of the history. Earlier duplicate detections are not proven fixed by these three clean runs.

## Historical run details

### Phase 1 — task foundation

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

### Phase 3 — recorded functional checks

| Check | Result | Observation |
|---|---|---|
| Level calibration | PASS | Calibration completed with the sensor level. |
| Pitch direction | PASS | Front-up mean +17.237 degrees; front-down mean -20.909 degrees. Both poses confirmed steady. |
| Roll direction | PASS | Left-side-up mean +23.206 degrees; right-side-up mean -19.256 degrees. Both poses confirmed steady. |
| Startup movement rejection | PASS | All 15 captured moving-stage rows remained uncalibrated. |
| Calibration after movement stops | PASS | Calibration completed after resting; 23 calibrated rows captured. |

Firmware checkpoint: `31a6b57ff01590cfad904c9e6d387c997ab0412c`. All 148 captured paired rows were fresh with read return code zero and missed counter zero. This does not establish worst-case execution time or exact sampling jitter.

### Phase 5 — original straight-driving build

Tester: Bryan (`xcher21`). Profile: HEIGHT_ONLY; commanded speed 50 mm/s; S2 encoders + S4 IMU + Safety Monitor. No line/barcode/network/obstacle task. Reference supplied by operator: 23 mm. Configured wheelbase: 110 mm; independent physical confirmation is not recorded here.

Tested build: `WT-e719d81481cc`, working snapshot `e719d81481cc06f953c69a5a0b04188ca2dd6d0f210d7394ab353f340d5fdeb7`, based on commit `ae2bfc2156178683bcd3e84fcec71a4bfe50802b`. These were working-tree tests; the base commit alone does not identify all test-image overrides. See [tested build manifest](logs/2026-11-06_tested_build.json) and [summary](logs/2026-11-06_three_trials_summary.json). Do not substitute a later documentation/code commit as the tested revision.

## Latest test-image traceability

Date: 2026-11-07 (Asia/Singapore). Profile: powered line following with S2 motors/encoders, S3 line task, S4 and S5 controller/Safety; maximum line speed 50 mm/s. Right tyre crosses; left tyre stays on the same flat floor. Measured wheel spacing 120 mm. Networking, barcode decoding and obstacle tasks were inactive in this image.

Working-tree base: `2d287d78ec02a107c0f16f6a31a3b4e557c78bef`; build `WT-609dab8d2247`. A base commit alone does not identify the image overrides.

Firmware SHA-256: `d240390f73fb1fcc6731a151acd96dd408eb0985afd0c0d2c0dc6844a30c865f`.
Runner SHA-256: `e9f9f26f4e0237d4ed653eb4cf49c9c6989fda1555974b87619af29d7eced063`.
Viewer SHA-256: `fa0c130eeb777552a32f9c24cf0a1461d24d576d51f42473d4ca392651cd34a3`.

## Evidence restoration and pending validation

The previous TEST_LOG and 70 captures temporarily removed during the latest-only cleanup are restored from the external backup. The latest three captures are preserved unchanged. Restored development captures retain their original content and are not automatically graded or reclassified. Any file already identical is left alone; conflicting files must be reviewed instead of overwritten.

The S4 source preparation preserves repository startup, S2/S3/S5 source, shared configuration and GPIO assignments. The normal repository configuration is not the same as the special test image; its full hardware integration remains to be retested. S5's default phase was 2 when reviewed. Do not label a later commit as the firmware tested above.

Pending: verify the normal repository build and full integration; assess other hump heights/contact conditions; investigate duplicate detections; test S4-H13 across different heights in one boot; measure complete timing/stack/RAM bounds. Prepared-source host/syntax checks do not establish those hardware claims. The prior normal-build attempt in the preparation environment stopped with Windows make exception `0xc0000008` before a complete link was verified.

Historical evidence-link corrections: the two Phase 1 public-copy filenames were unavailable; their links now point to the original board summary and build log restored from backup. The separately cited Phase 1 host capture `2026-10-27_213347_phase1_host.txt` could not be found in the available backup or workspace; its original PASS row is retained as a historical record, not reverified evidence.
