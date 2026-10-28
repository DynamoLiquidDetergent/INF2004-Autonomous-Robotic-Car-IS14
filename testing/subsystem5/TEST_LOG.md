# Subsystem 5 — test log

**Subsystem:** Spatial Optimizer (Obstacle Detection, Vehicle Controller, Safety Monitor)  ·  **Student ID:** 2503437  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-H01 to S5-H18 | scan maths: echo time to cm, servo pulse, integer square root, obstacle cluster, width, left/right shift, recommended action, fine-scan edge walk, packed i8 message round trip | FR38-FR41, NFR28, i8, minimum-bit rule | host (PC) | `sh testing/subsystem5/host/run.sh` | `[HOST] subsystem5 SUMMARY 20 passed, 0 failed` |
| S5-U01 | runs in a task context | | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | PWM block is out of reset (kernel leaves it in reset) | servo, echo timing | auto | `subsystem5_test.c` | `[TEST] S5-U02 PASS` |
| S5-U03 | servo slice 6 runs, 20 ms period | FR38 | auto | `subsystem5_test.c` | `[TEST] S5-U03 PASS` |
| S5-U04 | servo starts centred (1500 us) | FR38 | auto | `subsystem5_test.c` | `[TEST] S5-U04 PASS` |
| S5-U05 | Obstacle state fits in 3 bits | minimum-bit rule | auto | `subsystem5_test.c` | `[TEST] S5-U05 PASS` |
| S5-U06 | i8 profile message packs and unpacks | i8 | auto | `subsystem5_test.c` | `[TEST] S5-U06 PASS` |
| S5-U07 | i8 message is 8 bytes | minimum-bit rule | auto | `subsystem5_test.c` | `[TEST] S5-U07 PASS` |
| S5-U08 | two simultaneous fault bits are both kept | i14 (Tribunal 1, Buddy 4 question) | auto | `subsystem5_test.c` | `[TEST] S5-U08 PASS` |
| S5-U09 | clearing one fault bit with TWF_BITCLR leaves the other set | i14 | auto | `subsystem5_test.c` | `[TEST] S5-U09 PASS` |
| S5-P01 | phase 1: board runs with nothing wired, PWM out of reset | bring-up | manual | `S5_PHASE 1`, see `BRINGUP.md` | `alive` lines and `PWM out of reset = 1` |
| S5-P02 | phase 2: servo centres and sweeps 60-120 smoothly | FR38 | manual | `S5_PHASE 2`, see `BRINGUP.md` | smooth sweep, no strain |
| S5-P03 | phase 3: HC-SR04 supply + TRIG, ECHO not connected; divider tested at 3.3 V first | R5 | manual | `S5_PHASE 3`, see `BRINGUP.md` | divider midpoint 3.2-3.4 V; module cool |
| S5-P04 | phase 4: ECHO through divider, raw echo about 58 us per cm | NFR27 | manual | `S5_PHASE 4`, see `BRINGUP.md` | readings match a ruler |
| S5-M01 | ECHO LEVEL (do this BEFORE connecting GP17): power the HC-SR04, trigger it, measure the ECHO pin to GND with a multimeter or scope | R5, Week 6 Section 8.3 | manual | 1. Do not connect ECHO to the Pico. 2. Note the module supply voltage. 3. Measure the peak ECHO voltage. 4. Fit the 1k/2k divider if above 3.3 V and re-measure. | Peak at the Pico pin is 3.3 V or less |
| S5-M02 | servo sweeps 0 to 180 degrees without hitting its stops, 90 is straight ahead, index 0 is the robot's right | FR38 | manual | Run the normal image; watch the servo on `obstacle: ... deg` lines. Adjust `S5_SERVO_REVERSED` if backwards. | Smooth sweep, correct side |
| S5-M03 | raw echo counter changes with distance | NFR27 | manual | Console line `echo N us`. Hold a flat card at 10, 20, 50, 100 cm. | `echo` is about 58 us per cm |
| S5-M04 | distance against a ruler at 5 distances and 5 angles; detection rate 95% or better | NFR27 | manual | Table of ruler cm vs reported cm, 20 readings each. | Within 2 cm; at least 19 of 20 detected |
| S5-M05 | sensor unplugged: out-of-range vs no echo | Tribunal 1 test 5, Version C row 3 | manual | Remove the object, then unplug the sensor. | Object removed: `status 1` (out of range), no fault. Unplugged: `status 2`; after `S5_NOECHO_FAULT_N` readings `FAULT` is raised |
| S5-M06 | does the kit's sensor send a pulse when nothing is in range? | S5_NOECHO_FAULT_N | manual | Point at open space. | `status 1` with about 38000 us. If `status 2`, set `S5_NOECHO_FAULT_N` to 0 |
| S5-M07 | full scan time, detection to profile (R6, NFR31) | NFR31 | manual | Obstacle at 30 cm; read `scan N ms` on the PROFILE line. | Under 5 s total target; record each run |
| S5-M08 | width error against known objects (A5 narrow-object test) | NFR28 | manual | Objects 5, 10 and 20 cm wide at 20 and 40 cm. Calibrate `beam_comp_deg`. | Width within 20% (target 10%) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = runs on a PC, prints `[HOST]` lines;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-H01 to S5-H18 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_host.txt` | |
