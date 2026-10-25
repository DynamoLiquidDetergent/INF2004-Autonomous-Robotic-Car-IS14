# Team and subsystems

The tables are pre-filled with the **example** in `app_program/` (marked
*example*). Replace those rows with your own as you replace the example code.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | Network Gatekeeper: WiFi, MQTT, Telemetry | 2502578 | DynamoLiquidDetergent | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | *example:* Producer: makes a sample every 300 ms | 2500461 | SnakeMaister420 | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | IR Perception: Line Tracking, Junction Detection, Barcode Decode | 2500172 | ahdew | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | *example:* Monitor: prints reports, pauses the LED | 2500767 | xcher21 | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | | 2503437 | chongmh18 | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

Drafted table based off W6 & feedback to be confirmed by team OR ammended individually.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP1 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP2 | S4 | IMU SDA, I2C1 (Grove 2) |
| GP3 | S4 | IMU SCL, I2C1 (Grove 2) |
| GP4 | S2 | right encoder A (Grove 3) |
| GP5 | S2 | right encoder B (Grove 3) |
| GP6 | | free (jumper wire only, Grove 5 shares GP26) |
| GP7 | | free (jumper wire only, Grove 7 shares GP28) |
| GP8 | S2 | left motor M1A (PWM4 A) |
| GP9 | S2 | left motor M1B (PWM4 B) |
| GP10 | S2 | right motor M2A (PWM5 A) |
| GP11 | S2 | right motor M2B (PWM5 B) |
| GP12 | S5 | servo, servo port 1 (PWM6 A) |
| GP13 | | free (servo port 2) |
| GP14 | S2 | left encoder A (breakout header) |
| GP15 | S2 | left encoder B (breakout header) |
| GP16 | S5 | HC-SR04 TRIG (Grove 4). UNVERIFIED: 5 V supply source |
| GP17 | S5 | HC-SR04 ECHO (Grove 4). UNVERIFIED, do not wire: needs divider for 5 V echo |
| GP18 | | free (on-board RGB LED) |
| GP19 | | free |
| GP20 | S5 | START button (on board) |
| GP21 | S5 | STOP / reset button (on board) |
| GP22 | | free (on-board buzzer) |
| GP26 | S3 | IR left AOUT, ADC0 (Grove 6) |
| GP27 | S3 | IR centre AOUT, ADC1 (Grove 6) |
| GP28 | S3 | IR right / barcode AOUT, ADC2 (Grove 7). VBAT jumper left open (default) |

Last Updated: 24/10, 2350 hrs by Saad. 

**Changes from Week 6:** Left encoder moved from GP0/GP1 to GP14/GP15 to keep
the UART console free. GP20/GP21 added as START and STOP/reset buttons.
GP16/GP17 stay unverified until a 5 V source and ECHO divider are fully figured out.

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | | | |
| I2C1 | | | |
| SPI0 | | | |
| SPI1 | | | |
| ADC | S3 | Line Sensor Task (line_sensor.c) | GP26–GP28 |
| PWM slices (list) | | | |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 12–13 | *example:* led_task – 12 – any |
| 2 | 14–15 | *example:* producer_task – 14 – core 1 |
| 3 | 16–17 | line_sensor_task - 16 - any (High priority; 20 ms line cycle, 2 ms GP28 sub-cycle). barcode_decode_task - 17 - any (Medium priority; event-driven on i16 queue) |
| 4 | 18–19 | *example:* monitor_task – 18 – any |
| 5 | 20–21 | |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S2 → S3 | mailbox (owned by S3) | `mbx_samples` | `SAMPLE_MSG` in `subsystem2.h` | *example* |
| IF-02 | S3 → S2 | semaphore + fixed memory pool (owned by S2) | `sem_slots`, `mpf_samples` | receiver returns the block, then 1 slot | *example* |
| IF-03 | S3 → S4 | message buffer + event flag (owned by S4) | `mbf_summary`, `flg_events` | text line ≤ `S4_SUMMARY_MAX` bytes, then `FLG_BATCH` | *example* |
| IF-04 | S4 → S3 | shared data under a mutex (owned by S3) | `app_stats`, `mtx_stats` | `APP_STATS`, read only while locked | *example* |
| IF-05 | S3 → S5 | message buffer (owned by S3) | q_line_data | 	LINE_DATA_MSG (uint8_t): bits[7:5] pos_zone (0–7, 4=centre), bit[4] junction, bit[3] line_lost; sent every 20 ms | S3 + S5 (Teams, W8) |
| IF-06 | S3 → S3 (internal) | message buffer (owned by barcode_decode_task) | q_gp28_samples | GP28_SAMPLE_MSG (uint16_t): bits[15:4] 12-bit ADC value, bits[3:0] rolling slot index (gap in slot → reject decode); sent every 2 ms, drop-newest on full | S3 internal |
| IF-07 | S3 → S5 | message buffer (owned by S3) | q_barcode_cmd | BARCODE_CMD (uint8_t): 0=NONE, 1=TURN_LEFT (A), 2=TURN_RIGHT (B), 3=STRAIGHT (C), 4=UTURN (D); sent on valid decode only | S3 + S5 - A–D mapping TBD, confirm with S5 owner |
| IF-08 | S3 → S5 (Safety Monitor) | 	event flag bit (owned by Safety Monitor) | flg_fault_i14 | BS3 sets bit S3_FAULT_BIT (bit number TBD, agree with S5 owner) on debounced LINE_LOST outside bypass; never cleared by S3 | S3 + S5 - bit number TBD |
| IF-09 | S5 → S3 | function call / shared flag (owned by S3) | subsystem3_set_bypass(bool) | 	VC calls before leaving line during obstacle bypass; S3 suppresses LINE_LOST and fault-bit while active; VC clears on re-acquire or timeout | S3 + S5 |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are the example's usage.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 2 | 1 | |
| Semaphore | 16 | | 1 | | | |
| Event flag | 16 | | | | 1 | |
| Mailbox | 12 | | | 1 | | |
| Mutex | 12 | | | 1 | | |
| Message buffer | 12 | | | 3 | 1 | |
| Fixed memory pool | 12 | | 1 | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | 1 | | | | |
| Alarm handler | 12 | | | | 1 | |

(The kernel's USB and WiFi service tasks also use a few of these.)
