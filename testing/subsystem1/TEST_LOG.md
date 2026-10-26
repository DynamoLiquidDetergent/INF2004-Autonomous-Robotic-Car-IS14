# Subsystem 1 — test log

**Subsystem:** Network Gatekeeper (WiFi, MQTT, telemetry)  ·  **Student ID:** 2502578  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | Telemetry task is created at start-up | TEAM.md sec 4 | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | Telemetry task sleeps between ticks (not busy-looping) | NFR4 (S1 never delays driving) | auto | `subsystem1_test.c`: sample task state 10 times, 97 ms apart | `[TEST] S1-U02 PASS` (≥ 9 of 10 waiting) |
| S1-U03 | 100 ms loop does not drift | Telemetry period basis (FR telemetry 500 ms) | manual | 1. Flash normal image. 2. Log console for ~10 s. 3. Check `t=` difference between consecutive `s1: alive` lines | 10 consecutive lines, every difference 1000 ms ± 1 ms |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-26 | S1-U01 | `<c27b799>` | Saad | PASS | `logs/S1-U01-U02_26-10-2026.txt` | TEST=1 image |
| 2026-10-26 | S1-U02 | `<c27b799>` | Saad | PASS | `logs/S1-U01-U02_26-10-2026.txt` | TEST=1 image |
| 2026-10-26 | S1-U03 | `<c27b799>` | Saad | PASS | `logs/S1-U03_26-10-2026.txt`, `logs/S1-U01-U02_26-10-2026.txt` | 1000 ms ±0 over 10+ ticks; also held during S2 timing tests |