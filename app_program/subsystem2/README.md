# Subsystem 2: Motion Control

Owner: Aaron Alison Dsilva (2500461). Drives both motors, reads both wheel
encoders, and runs one PI speed loop per wheel at 50 Hz.

## Files

| File | What it does |
|---|---|
| `subsystem2.h` | Public interface: command, status and odometry formats, priorities, pins, settings to calibrate |
| `subsystem2.c` | `subsystem2_init()` / `subsystem2_start()` |
| `motion_control.c` | `motion_task` (priority 6, core 1, every 20 ms) and its cyclic handler |
| `motion_math.c/.h` | Pure maths: speed from encoder period, PI, ramp, turn arc, epochs. Tuning values are at the top of the `.h` |
| `drv_motor.c/.h` | PWM on GP8–GP11 (slices 4 and 5, 20 kHz) |
| `drv_encoder.c/.h` | Encoder pins GP14/GP15 (left), GP4/GP5 (right), and the shared GPIO interrupt |
| `task_producer.c` | Template example, still used by subsystems 3 and 4. Remove later |
| `subsystem2_test.c` | On-target tests (`make TEST=1`) |

PC tests of the maths: `sh testing/subsystem2/host/run_host_tests.sh`

## For the Vehicle Controller (subsystem 5)

Send one 32-bit command word at least every 100 ms while the robot should move:

```c
#include "../subsystem2/subsystem2.h"

UW cmd = s2_cmd_pack(S2_MODE_DRIVE, 200, steer_mmps, epoch);   /* 200 mm/s forward */
tk_snd_mbf(q_motion_cmd, &cmd, sizeof(cmd), TMO_POL);
```

| Mode | speed | param |
|---|---|---|
| `S2_MODE_STOP` | – | – |
| `S2_MODE_DRIVE` | mm/s, signed | steering mm/s (+ = turn right). 0 = hold a straight line |
| `S2_MODE_TURN` | wheel speed (0 = default 150) | degrees, + = clockwise. Turns on the spot |
| `S2_MODE_DRIVE_DIST` | mm/s, sign = direction | distance in mm, 0–1023 |
| `S2_MODE_HOLD` | – | keep the current speed steady (e.g. over a hump) |

- **Epoch** (0–15): add 1 on every pause, fault and reset. Older epochs are thrown away.
- **No command for 100 ms** stops the motors (state `TIMEOUT`).
- Repeating the same TURN or DRIVE_DIST word only keeps it alive; it does not start a new move. Send STOP in between, or use a new epoch.
- Read the result with `s2_get_status()`: speeds, state, `done` (TURN / DRIVE_DIST finished) and the newest accepted epoch.

## For the Safety Monitor (subsystem 5)

Call once in `subsystem5_init()`, after creating the flags:

```c
subsystem2_bind_safety(flg_inhibit, INHIBIT_BIT, flg_fault_i14, 0x1U);
```

- While the inhibit bit is set, Motion Control keeps PWM at 0 and discards every command (checked before every PWM write).
- After the inhibit clears, only a command with a **newer epoch** moves the robot.
- On a motor stall Motion Control sets bit 0 of `flg_fault_i14` and stops. It never clears that bit.

## For the IMU / terrain task (subsystem 4)

`s2_get_odometry()` returns distance (mm, 16 bits) and a timestamp (ms, low 16 bits of `tk_get_tim`), updated every 20 ms. Both wrap, so always use differences:

```c
UW now = s2_get_odometry();
UH moved_mm = (UH)(s2_odo_dist_mm(now) - s2_odo_dist_mm(before));
```

## For the HC-SR04 echo (subsystem 5)

The RP2040 has one interrupt for all GPIO pins, and subsystem 2 owns its handler. To get edges on GP17:

```c
s2_gpio_hook_register(17U, S2_GPIO_EDGE_RISE | S2_GPIO_EDGE_FALL, echo_hook);
```

`echo_hook(gpio, events, t_us)` runs in interrupt context: store the timestamp, set a flag, return.

## Before the robot drives: calibrate

Set these at the top of `subsystem2.h` from the bench tests in `testing/subsystem2/TEST_LOG.md`:

| Setting | Test |
|---|---|
| `S2_ENC_QUADRATURE` | S2-M01 |
| `S2_ENC_PULSES_PER_REV`, `S2_WHEEL_DIAMETER_MM`, `S2_TRACK_WIDTH_MM` | S2-M02 |
| `S2_MOTOR_L_INVERT`, `S2_MOTOR_R_INVERT` | S2-M03 |
| gains in `motion_math.h` | S2-M04 |

## Known limits

- With 20 pulses per turn, one pulse is about 10 mm of travel and about 10° of a turn on the spot. Turns and DRIVE_DIST stop at the nearest pulse, so the error is up to half a pulse (about ±5°) plus coasting. If S2-M08 fails, count both edges of channel A or use quadrature.
- Speeds below about 40 mm/s read as 0, because pulses are then more than 250 ms apart.
