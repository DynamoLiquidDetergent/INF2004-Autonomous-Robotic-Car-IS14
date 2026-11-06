# S4 + S5 height integration

Prepared from S5 branch `b5281825af17b0bead43c05d1c4f35e9eabbc7b6`
and Bryan's S4 Phase 5 `2309ef1d83444f5292fb572e90e20ebd39965c26`.
Work on `s1-s3-s4-s5-integration`; keep the original branches available.

## Connections implemented in this branch

S5 initialisation binds a bounded reader before any tasks start. S4 reads S2's
existing `s2_get_odometry()` word (high 16 bits cumulative mm; low 16 bits kernel
milliseconds). Actual S2 `now_ms()` and this reader both use `tk_get_tim()`.
There is no conversion to a microsecond timestamp and no extra odometry queue.

The reader takes odometry/status/odometry once. A changing pair is skipped,
never spun on. These independently published words are not a transactional
snapshot; clock/data guards reduce risk but exact sample alignment still needs
physical validation. First input establishes a baseline. Source age <=40 ms,
update gap <=60 ms, normal forward distance increment <=50 mm, ordinary wraps,
unchanged duplicates and epoch changes are checked. These are the existing
Phase 5 development guards, not newly validated operating requirements.

Forward/non-reversing distance is derived from successive signed modular
encoder-distance differences. TURNING/INHIBITED/STALLED states and negative
reported wheel speeds block forward use. IDLE/TIMEOUT with nonnegative speeds
can supply a real stationary baseline or manually pushed forward distance;
there is no fake distance, assumed speed, or unconditional forward flag.
An average of the wheel counts cannot prove that each individual wheel is
healthy; the count reliability caveat in S2's motion_control.c still applies.
Validate measured distance before claiming height accuracy.

S4 retains its signed axis mapping: components up, two-hole edge forward.
GP2 SDA/GP3 SCL, 20 ms period, priority 18 and any-core affinity are unchanged.
S5 still owns motor inhibition and fault clearing. S4 sets/reasserts only its
assigned IMU bit in S5's existing `flg_fault_i14` on local FAULT or BAD timebase.
S4 never clears this group. A persistent latched S4 fault requires a board
restart after the cause is corrected; no automatic sensor recovery is claimed.

One S3 header conflict was corrected on this integration branch: S3_FAULT_BIT
is bit 1, matching S5 FLT_LINE_BIT; S4 uses S5 FLT_IMU_BIT (bit 2). The old S3
macro assigned bit 2 too. The current line task does not use that macro, but
leaving duplicate ownership would make later fault reporting ambiguous. A
compile-time cross-check prevents the conflicting definitions returning.

S4 publishes its historical highest valid completed height through one aligned
32-bit local snapshot. S5 uses its existing `hump_mm` input and S1 telemetry
layout (0..254 mm, 255 unavailable). This reuses the existing clamp: larger
values are clipped for telemetry, while S4 serial output keeps the full value.
It is not a new range/precision claim or the full Phase 6 terrain-report codec.

## Create and review the integration branch in VS Code PowerShell

Save your work, then inspect `git status`. Commit intended tracked code/test
changes before switching; retain unrelated untracked files separately. The
helper refuses tracked edits or conflicting files and never resets/stashes them.

```powershell
cd "C:\Users\bryan\INF2004-Autonomous-Robotic-Car-IS14"
git status
git fetch origin
git switch -c s1-s3-s4-s5-integration origin/s5-integration

$integrationPython = "C:/Users/bryan/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe"
$integrationScript = "C:/Users/bryan/.codex/.chatgpt-projects/g-p-68e3356774448191a3683376ae7e0df9/output/manage_s4_s5_integration.py"

& $integrationPython $integrationScript --check
& $integrationPython $integrationScript --apply
git diff --check -- . ':(exclude)testing/subsystem4/logs/**'
git diff --stat
git diff -- app_program/subsystem3/subsystem3.h app_program/subsystem5
```

The check is pinned to the reviewed S5 commit. If the team has advanced that
branch, stop with the reported hash and prepare a fresh patch; do not reset
backwards or force the patch. If your new branch already exists, inspect it
instead of deleting/recreating it. The helper imports only S4-owned code/tests
and the reviewed S3/S5 glue; it does not merge older shared settings from S4's
branch over the team's newer work.

The whitespace check excludes historical raw logs, which are imported unchanged
as evidence. New source and Markdown changes are still checked.

Review the diff, then run software checks and commit the new checkpoint:

```powershell
& $integrationPython $integrationScript --host
git add -- app_program/subsystem4 testing/subsystem4 app_program/subsystem3/subsystem3.h app_program/subsystem5/subsystem5.c app_program/subsystem5/vehicle_controller.c
git diff --cached --stat
git commit --only -m "feat(s4): integrate IMU height input and S5 reporting" -- app_program/subsystem4 testing/subsystem4 app_program/subsystem3/subsystem3.h app_program/subsystem5/subsystem5.c app_program/subsystem5/vehicle_controller.c
```

The host checks include ten suites; they do not move a motor or establish
sensor/height accuracy. Commit only after reviewing their actual result.

## Choose a physical mode

Both are supplied. Since Bryan's car already drives itself, use the powered
procedure below. Hand-push is an optional diagnostic alternative, with separate
mode-labelled evidence. Do not mix the two modes into one acceptance result.

The board's switch stops everything, so the powered test uses a software stop
and saves evidence BEFORE requesting power-off. It adds no motor-command task:
S5's existing Vehicle Controller remains the only command sender. The test-image
guard uses its existing latched ABORT decision; the Safety Monitor remains active.
`S4_POWERED_BENCH_KIND` is 0 in committed source, so normal builds do not acquire
this bench-stop behaviour. The image helper explicitly selects kind 1 or 2.

## Motor-powered procedure for this car

Prepare a clear straight track with the team's working line-sensor setup.
For height, the line must continue over the hump and onto flat ground beyond it.
Avoid barcodes and junctions during these trials. Fix the IMU to the chassis,
components up and two-hole edge forward. Keep the ultrasonic sensor connected
and facing forward. Leave enough loose USB cable for the route. Do not disable
obstacle handling or a fault to obtain a result.

Each run starts automatically after the team's line-startup/calibration and
placement countdown. Keep the car flat and still so S4 can calibrate. The line
sensor retains its default range when the startup sweep lacks sufficient span;
use the team's already validated line setup. The tester does not change line
calibration values. If a fresh line sweep is needed, validate that setup first
and restart for the scored S4 run.

First validate real encoder distance on flat ground. The DISTANCE image requests
a stop after approximately 1000 mm of odometry. Then independently measure the
ACTUAL start-to-rest displacement of the same marked chassis point, including
coasting. The requested 1000 mm is not treated as a measured distance.

```powershell
& $integrationPython $integrationScript --build --profile distance
& $integrationPython $integrationScript --flash

$poweredTest = "C:/Users/bryan/.codex/.chatgpt-projects/g-p-68e3356774448191a3683376ae7e0df9/output/run_powered_height_test.py"
& $integrationPython $poweredTest --case distance
```

At flashing, the explicit POWERED image prompt indicates automatic motor startup;
keep the car supported with wheels clear while flashing. After copying, turn it
off before arranging a floor run. The tester asks you to leave it OFF at the
marked start, then prints `SWITCH ON / RESET NOW`. Switch on/reset at that cue.
It connects during startup and captures the run without further keyboard input.
If the switch leaves the Pico powered by USB, use the board's RESET/RUN control
at that cue as well, because the previous run's ABORT is latched. Do not use
BOOTSEL to restart a scored run.

Let S5 drive and stop the car. Wait for `CAPTURE SAVED`, then measure its actual
travel and answer the setup confirmation. The car must remain at the endpoint
until measured. Wait for the final `SAVED` line before switching off. A passed
distance record for this same code commit is required before the powered height
script proceeds. It is one +/-5% sanity check, not complete S2 distance validation.

Then build the one-hump image and run three predeclared trials:

```powershell
& $integrationPython $integrationScript --build --profile drive
& $integrationPython $integrationScript --flash
& $integrationPython $poweredTest --case height
```

Enter the hump's independently measured base-to-peak height and measurement notes.
For each of three trials, reposition with power OFF, then switch on/reset only
at the cue. S5 follows the straight line over the hump and requests ABORT after
the first completed terrain traversal. It also requests a stop after 3000 mm
of odometry or 30 seconds if the expected hump is not completed. These are test
limits; hardware stopping distance and worst-case response are unmeasured.

The tester waits for a stationary, healthy LEVEL endpoint and saves the trace
before asking whether the physical traversal was performed correctly. Never
push, hand-tilt or lift the car to force a state during a powered trial. Turn off
at the next setup prompt or after the final `SAVED`, then reposition for the
next fresh boot. A power loss/reset inside a scored trial produces an incomplete
or failed record. If the car does not stop or leaves the safe test area, use
the physical power switch immediately; preserve the partial result.

All three proper traversals require an ordered LEVEL -> CLIMBING -> DESCENDING
-> LEVEL sequence, exactly one completed hump, a valid event height and <=10%
error on each. LIMIT/TIMEOUT stops cannot earn a hump PASS. The one-hump image
checks the highest snapshot within that run; because each trial reboots, it
does not demonstrate retaining a maximum across several physical humps in one
mission. That additional physical behaviour remains untested.

## Optional diagnostic procedure: hand-push the complete car

Use the assembled car with both encoders connected. Secure the IMU in its final
pose; it must tilt with the chassis. This is a whole-car traversal, not the old
cardboard-pose experiment. Check wiring with power disconnected.

```powershell
& $integrationPython $integrationScript --build --profile push
& $integrationPython $integrationScript --flash
```

The build helper makes a scratch image from your committed branch. It records
three explicit image-only settings: S5_PHASE=5 (Obstacle Detection + Safety
Monitor, no Vehicle Controller), VC_AUTO_START=0, and an S4 build/profile tag.
WIFI=none, NET=none, MQTT=0, TEST=0, SMP=1, USB console. The repository's S5
bring-up default stays at 2; the image override is written down in metadata.
S3's drive demo remains off. With no drive sender, S2's 100 ms command timeout
keeps motor outputs stopped; only real wheel motion supplies odometry.

At flashing: close other serial monitors, disconnect other Picos, keep the
sensor connected and the whole car level. Follow the BOOTSEL/RPI-RP2 prompt.
The helper verifies one RP2040 drive and copies the new image. This replaces
the Pico's existing program. Keep the car still after restart for calibration.

Keep the ultrasonic sensor connected and pointing into clear space so a scan
or missing-sensor fault does not interrupt this measurement. The Safety Monitor
is active; a fault/inhibit invalidates a traversal and must be reviewed.

Measure the printed hump's vertical height from its flat base to peak using a
ruler/caliper, in mm. Note the measurement uncertainty. Then run:

```powershell
$heightTest = "C:/Users/bryan/.codex/.chatgpt-projects/g-p-68e3356774448191a3683376ae7e0df9/output/run_integrated_height_test.py"
& $integrationPython $heightTest
```

This tester captures real serial and asks for the reference height; no preset
height is supplied. It checks the stamped firmware and calibration/timebase,
then guides a flat-distance check and three predeclared hump traversals. It
records actual outcomes and preserves earlier logs. It does not commit/push.

1. On a flat surface, mark 1000 mm between two chassis reference positions.
   At the cue, push forward from the first mark to the second, straight and
   without lifting/slipping wheels. Do not use the sensor position as one mark
   and a wheel position as the other. Stop at the second mark, then confirm.
   The displayed odometry difference should match within the existing S2
   distance target of +/-5%. This one sanity check is not the full S2-M06
   multi-speed validation. A fresh timestamp alone does not prove good distance.
2. Put the complete car flat before the school hump; keep wheels in contact.
   Confirm readiness. Push forward smoothly, starting on the flat approach,
   over the hump and onto flat ground beyond it. Keep a moderate steady pace
   so the complete CLIMBING -> DESCENDING -> LEVEL sequence fits the existing
   10 s traversal timeout. Do not lift an edge by hand to force a state.
3. At the completion, leave the car flat and still. The tester records the
   event's `height_mm`, not a value typed in by you. Reposition for the next
   trial only at the setup prompt; backward repositioning is excluded from
   the scored forward window and can invalidate that earlier attempt's height.
4. Repeat all three trials. Actual error is
   `abs(estimate-reference)/reference*100`; each validly performed trial needs
   a valid estimate and <=10% error for H10 PASS. A proper traversal with no
   height or excessive error is FAIL. A declared physical setup mistake is
   INVALID SETUP, preserved with the run; do not relabel a device failure.

If the chassis bridges the hump without exceeding the current pitch thresholds,
the traversal may not be detected. If encoder distance freezes/cancels, height
may be invalid or inaccurate. Neither is fixed by changing the reported result.
The approximate distance*sin(pitch) model and pitch filter still require this
physical acceptance test. No +/-10% achievement is supplied in advance.

## Review and push

Review TEST_LOG.md, the printed summary JSON and raw trace, including any FAIL,
INCOMPLETE or INVALID SETUP. Commit actual results separately. The S1 dashboard
requires the team's network/MQTT build; a WiFi-off bench image will not publish
there. On a later network-enabled phase 6+ image, S5's existing highest-hump
field is connected by this code, with the existing 254 mm telemetry clamp.

```powershell
git status
git diff --check -- . ':(exclude)testing/subsystem4/logs/**'
git add -- testing/subsystem4
git commit --only -m "test(s4): record integrated height test results" -- testing/subsystem4
git push -u origin s1-s3-s4-s5-integration
```

Keep this on the new integration branch until reviewed; no main/S5 branch push
or pull request is performed by the helpers.

The powered and hand-pushed bench images keep WiFi off. Dashboard control is
not available in these images. Full mission builds keep the bench guard off
and use the team's usual network/start/stop settings.
