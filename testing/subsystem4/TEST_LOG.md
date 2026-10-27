# Subsystem 4 â€” test log

**Subsystem:** Monitor (example)  Â·  **Student ID:** <student ID>  Â·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | message buffer max line = S4_SUMMARY_MAX | IF-03 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | LED pauses and resumes every 6 reports | IF-05 | manual | 1. Flash normal image. 2. Watch LED and console ~20 s. | console shows "LED task paused"/"resumed"; LED stops/starts |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone appâ€¦).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_â€¦.txt` | |


## IMU replacement work

Example S4-U01/U02 are retired when their example behaviour is removed; IDs are not reused. Existing result rows are unchanged.

| ID | What is tested | Method | Expected result |
|---|---|---|---|
| S4-U03 | Task lifecycle, absolute deadline, overrun and wrap | Host assertions; board task query/priority check; ten-minute heartbeat observation | Task at 18; progress with existing tasks; misses recorded. Precise jitter and execution time not measured |

Hardware runs for these new cases: NOT RUN. Append actual results with tested commit and evidence; do not overwrite earlier rows.
