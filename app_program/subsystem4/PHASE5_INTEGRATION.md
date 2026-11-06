# Phase 5: hump height from pitch and odometry

This is S4 implementation code, not evidence of measured height accuracy or
completed S2/S5 integration. Phase 5 test files and hardware results will be
prepared separately after this code checkpoint is pushed.

## Existing behaviour preserved

The Phase 4 terrain FSM and event counters are unchanged. Sampling remains
20 ms / 50 Hz, priority 18, any core. GP2 SDA / GP3 SCL on I2C1 are unchanged.
The two-hole edge is forward, components up; the existing signed axis mapping
is retained. Magnetic heading remains disabled pending validation.

## Height model

`s4_odometry.c` estimates vertical rise using successive cumulative-distance
increments and the average sine of their pitch estimates:

    vertical_increment_mm = distance_increment_mm *
                            (sin(previous_pitch) + sin(current_pitch)) / 2

Distance must represent travel along the path, not horizontal displacement.
Wheel travel and chassis pitch are only an approximate model of the hump
surface; wheelbase, slip, pitch filtering and timestamp skew can affect the
result. The highest cumulative rise is the candidate hump height.

A provisional estimate starts as positive pitch leaves the 2-degree level
band, using the preceding level sample. This reduces loss from waiting for
the 6-degree climbing threshold and three-sample FSM debounce; it does not
eliminate onset or filtering error. Only a completed Phase 4 hump sequence
can publish a height. A downhill-only or aborted sequence cannot do so.
Highest height is updated only from a valid completed estimate and lasts
until the S4 task is reinitialised (currently a controller restart).

No odometry, unsafe direction, invalid/stale IMU data, stale/gapped/jumping
odometry, a clock-format change, missing level baseline or bad timebase
consistency makes that traversal's estimate invalid. Subsequent good samples
cannot repair a partially observed traversal. No movement cannot produce a
valid height. The existing terrain hump counter can still increase while
height is unavailable; detection and height validity are separate.

## Proposed odometry binding (Week 6 i19)

The checked branch still contains S2's example producer and S5's empty
template. There is no implemented live odometry provider on this branch.
S4 therefore starts unbound and reports `height_valid=0`, `height_mm=-1`.
No simulated distance, assumed driving speed, new queue or new IF ID is used.
Existing TEAM.md IF-17 is a proposal, not approval of this callback/layout.

S4 exposes the local C callback type `s4_read_odometry_fn` in `s4_odometry.h`.
Buddy 5 can bind a reader with `subsystem4_bind_odometry(reader)` during
`subsystem5_init()`, before any tasks start on either core. Runtime binding
is rejected. The reader executes in the S4 task and must be bounded and
nonblocking, returning a coherent copied snapshot. It must not consume a
message also needed by another receiver, drive motors, or modify S4 state.
The callback ABI is not a packed message and its `sizeof` is not a wire size.

Two proposed formats are supported explicitly:

| Format | Populate `s4_odometry_t` | Matching clock |
|---|---|---|
| Tribunal proposal | `uint16_t distance_mm`, `uint32_t timestamp`, `clock_format=S4_ODOM_US32` | Same microsecond hardware clock used by S4; do not substitute an unrelated epoch |
| Current TEAM.md IF-17 proposal | `s4_odometry_decode_word(word, confirmed_forward, &out)`; high 16 bits cumulative mm, low 16 bits timestamp | Low 16 bits of kernel `tk_get_tim()` in milliseconds |

The provider must set/return invalid when the source is stale, direction is
unknown/reverse, or a controller reset changes the odometry origin. The word
contains no direction or validity bits: obtain that information separately
from the Vehicle Controller; never assume `forward=true`. Turns in place
and reversing are outside this non-reversing height model.

The low-16 timestamp wraps every 65.536 seconds and distance wraps every
65.536 metres. Unsigned differences handle ordinary single wraps. Provider
freshness/origin guarantees are still required: a short timestamp alone
cannot distinguish an arbitrarily old/replayed value across a full wrap.
A changing distance with an unchanged timestamp is rejected. Repeated
unchanged samples do not reintegrate distance or overwrite the original
pitch associated with that snapshot.

Development guards are **not confirmed measurement requirements**:
40 ms maximum snapshot age, 60 ms maximum interval, 50 mm maximum distance
increment, and 10 s maximum candidate lifetime. Confirm these against
Buddy 2's measurement resolution, update rate and operating speed. There
is no numeric bit allocation for height yet; agree range and precision in
Phase 6 after measurements.

Pitch is associated with the current S4 read when a new odometry snapshot
arrives. This is bounded-age sampling, not interpolation to an exact common
sample time. Timestamp alignment and its height error require validation.

## Diagnostics and timing dependency

Existing `S4 ...` fields remain, followed by:

| Field | Meaning |
|---|---|
| `bus_rc` | I2C backend initialisation result |
| `init_rc` | Sensor initialisation result; -100 means not attempted after bus failure |
| `timebase` | PENDING, CONSISTENT or BAD |
| `odom_valid` | Current snapshot passes input/age/interval checks with fresh IMU data |
| `height_valid`, `height_mm` | Last completed estimate validity and rounded mm; -1 when unavailable |
| `highest_valid`, `highest_mm` | Highest valid completed estimate since task initialisation; -1 when unavailable |
| `height_reason` | Current traversal/unavailable-estimate diagnostic; OK for a valid completed estimate |

Sensor init codes come from `drv_motion_sensor.c`: -1 invalid bus, -2 identity
read failed, -3 unexpected identity, -4 configuration write failed, -5
accelerometer readback failed. Existing loop `rc=-1` remains a general
communication/not-initialised result. These diagnostics do not fix the
unresolved Robo Pico sensor FAULT.

`s4_clock_check.c` compares hardware microsecond progress with kernel
milliseconds over at least 200 ms. Tolerance is 10% of elapsed time plus
10 ms to allow tick quantisation/read skew. BAD is latched until restart;
intervals over 1 s also block the check conservatively. Both height formats
are blocked unless the comparison is CONSISTENT. This catches gross scaling
errors like the observed ~23,437 timer counts per wall second, but cannot
prove calibrated timing, exact timestamp alignment or independent clock
accuracy. No timer/watchdog register or shared kernel configuration is
changed. Resolve a BAD result with the team before height acceptance tests.

S4 still only reports a local FAULT. Binding the existing Safety Monitor
event group and defining packed outgoing reports remain Phase 6 work.
No new kernel objects, safety responses or shared fault variables are added.

The existing live demo can display the additional raw fields without
changing its Python code. This firmware change does not create log files
or modify `TEST_LOG.md`. The physical +/-10% height-error requirement,
20 ms processing budget, RAM/stack use and inter-task behaviour remain
unverified until the subsequent tests and actual team integration.
