# Subsystem 4 — test log

**Subsystem:** Physics Watchdog — IMU-Based Motion and Terrain Monitoring · **Student ID:** 2500767 · **Owner:** Bryan (`xcher21`) · **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Host tests check portable software logic; manual/board tests establish only their recorded hardware scope.

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
| S4-H12 | Powered hump detection and completion | Phase 5 | manual | Check LEVEL → CLIMBING → DESCENDING → LEVEL and retained height/highest during a powered crossing. | One completion for one physical crossing; profile-specific stop/continue behaviour recorded separately. |
| S4-H13 | Highest valid hump in one boot | Phase 5 | manual | Traverse at least two independently measured humps of different heights without resetting | Highest stored equals the largest valid estimate and does not decrease on the smaller hump |
| S4-H15 | Firmware bias-corrected right-wheel height | S4 height output | manual | Powered crossing of a 23 mm reference; 120 mm baseline configuration; | Reported height within 20.7–25.3 mm, valid completion and retained highest; one-trial scope only. |

*Method:* **auto / host** = software assertions on the indicated platform; **manual / board** = physical observations with saved evidence. Synthetic model checks are not physical accuracy results.

## Results

Phase 1–4 records are retained. The active height-test record below contains the latest corrected-firmware trial only. Earlier height-development captures were removed from this active set during cleanup; this does not establish that those earlier runs passed. Add a row for each subsequent trial.

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
| 2026-11-08 | S4-H15 | `BIAS-2026-11-08_152149_900751` (test-image snapshot) | Bryan | PASS | [Capture](logs/2026-11-08_152328_818699_buddy4_handover.txt); [Review](logs/2026-11-08_152328_818699_buddy4_handover_reviewed.json) | Hump event at 15:23:44 SGT; reported/stored height 23 mm, reference 23 mm; 0.0% error at whole-mm telemetry resolution. |
| 2026-11-08 | S4-H12 | `BIAS-2026-11-08_152149_900751` | Bryan | PASS (one-event functional scope) | [Capture](logs/2026-11-08_152328_818699_buddy4_handover.txt) | CALIBRATING → LEVEL → CLIMBING → DESCENDING → LEVEL; one recorded hump event; highest 23 mm retained. Profile continues driving. No sensor read error, missed release or reported fault. No junction captured; crossing behaviour not revalidated. |
| 2026-11-08 | S4-H10 | `BIAS-2026-11-08_152149_900751` | Bryan | PARTIAL (1 of 3 trials) | [Capture](logs/2026-11-08_152328_818699_buddy4_handover.txt) | This corrected-height trial is within ±10%; two further trials needed for three-run repeatability. No general uncorrected accuracy or other-height calibration claim. |
| 2026-11-08 | S4-H13 | `BIAS-2026-11-08_152149_900751` | Bryan | NOT RUN | [Capture](logs/2026-11-08_152328_818699_buddy4_handover.txt) | Highest retained after one hump; different-height humps in one boot not tested. |

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

