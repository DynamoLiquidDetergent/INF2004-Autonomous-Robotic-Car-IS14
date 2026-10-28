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
| S1-U04 | Empty outage buffer: pop fails, count 0 | IF-15 | auto | make TEST=1, flash, read console | `[TEST] S1-U04 PASS` |
| S1-U05 | 20 pushes: count 20, 0 dropped, FIFO order 1...20 | IF-15 (20 frames) | auto | make TEST=1, flash, read console | `[TEST] S1-U05 PASS` |
| S1-U06 | 25 pushes: oldest 5 dropped, 6...25 kept in order | IF-15 (drop-oldest) | auto | make TEST=1, flash, read console | `[TEST] S1-U06 PASS` |
| S1-U07 | Bad frames refused. too-small pop keeps the frame | IF-15 | auto | make TEST=1, flash, read console | `[TEST] S1-U07 PASS` |
| S1-U08 | Command parser accepts PAUSE, RESUME, ABORT and decodes them. Reject count stays 0 | FR7, IF-15 RX | auto | make TEST=1, flash, read console | `[TEST] S1-U08 PASS` |
| S1-U09 | NULL, empty and 2-byte frames rejected and counted. A valid frame afterwards still accepted | FR8, IF-15 RX (1 byte) | auto | make TEST=1, flash, read console | `[TEST] S1-U09 PASS` |
| S1-U10 | Codes other than 1, 2 & 3 rejected. (0, 4, 255, ASCII 'P') means reject count is 4 | FR8 | auto | make TEST=1, flash, read console | `[TEST] S1-U10 PASS` |
| S1-U11 | Fixed inputs pack to exact expected bytes (telemetry 15 B, heartbeat 4 B, event 3 B); expected bytes computed independently of the C code | CR1, IF-15 TX | auto | make TEST=1, flash, read console | `[TEST] S1-U11 PASS` |
| S1-U12 | Telemetry frame is 15 B. Every field round-trips at its min & max | CR1 | auto | make TEST=1, flash, read console | `[TEST] S1-U12 PASS` |
| S1-U13 | Out-of-range values clamp to the field max (never to "no data"); letter past Z becomes NONE; all no-data values round-trip | CR1 | auto | make TEST=1, flash, read console | `[TEST] S1-U13 PASS` |
| S1-U14 | Heartbeat is 4 B; round-trips at min & max; rejected count stops at 15 | CR1, IF-15 TX | auto | make TEST=1, flash, read console | `[TEST] S1-U14 PASS` |
| S1-U15 | Event is 3 B; type & data round-trip at limits; type > 7 or data > 13 bits refused | CR1, IF-15 TX | auto | make TEST=1, flash, read console | `[TEST] S1-U15 PASS` |
| S1-U16 | Header carries version & type; replayed bit set without changing anything else; NULL or too-small buffer refused, nothing written; wrong frame type refused on unpack | CR1, IF-15 TX | auto | make TEST=1, flash, read console | `[TEST] S1-U16 PASS` |
| S1-U17 | Network baseline with template code (no S1 changes to mqtt_utk.c): Pico joins 2.4 GHz hotspot, connects to laptop Mosquitto broker; heartbeat reaches mosquitto_sub; messages from mosquitto_pub reach the Pico; s1 task timing unaffected by WiFi | IF-15 (baseline before change), NFR4 | manual | build `WIFI=cyw43 NET=lwip MQTT=1`; `mosquitto -c test.conf -v`; `mosquitto_sub -h localhost -t "#" -v`; `mosquitto_pub -h localhost -t mtk3smp-pico/heartbeat -m hello` | Pico: `connected`, `subscribed`, TX/RX `hb N`, RX `hello`; mosquitto_sub: `hb N` and `hello`; `s1: alive` stays 1000 ms apart |
| S1-U18 | Valid map (F=K, U=D, L=A, R=Z; bytes 6A 80 0C) accepted; stored word = frame + valid bit; lookup K/D/A/Z gives F/U/L/R; unmapped letter B and an invalid map give nothing | Prof mapping req., FR7, IF-18 | auto | make TEST=1, flash, read console | `[TEST] S1-U18 PASS` |
| S1-U19 | NULL and 0/2/4-byte map frames rejected and counted; word left at 0 | FR8, IF-15 RX | auto | make TEST=1, flash, read console | `[TEST] S1-U19 PASS` |
| S1-U20 | Letter code 26 (FORWARD) or 31 (RIGHT), or a spare bit set, rejected | FR8 | auto | make TEST=1, flash, read console | `[TEST] S1-U20 PASS` |
| S1-U21 | Same letter used twice rejected; valid map outside READY rejected; each rejection maps to the right MAP_ACK reason | FR8, IF-18 | auto | make TEST=1, flash, read console | `[TEST] S1-U21 PASS` |
| S1-U22 | IF-18: no valid map at boot; stored word reads back unchanged; s1_nav_lookup uses it; store cleared after the test | IF-18, CR1 (single store) | auto | make TEST=1, flash, read console | `[TEST] S1-U22 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-26 | S1-U01 | `<c27b799>` | Saad | PASS | `logs/2026-10-26_S1-U01-U02.txt` | TEST=1 image |
| 2026-10-26 | S1-U02 | `<c27b799>` | Saad | PASS | `logs/2026-10-26_S1-U01-U02.txt` | TEST=1 image |
| 2026-10-26 | S1-U03 | `<c27b799>` | Saad | PASS | `logs/2026-10-26_S1-U03.txt`, `logs/2026-10-26_S1-U01-U02.txt` | 1000 ms ±0 over 10+ ticks; also held during S2 timing tests |
| 2026-10-26 | S1-U04 | `<609dd97>` | Saad | PASS | `logs/2026-10-26_S1-U04-U07.txt` | TEST=1 image |
| 2026-10-26 | S1-U05 | `<609dd97>` | Saad | PASS | `logs/2026-10-26_S1-U04-U07.txt` | TEST=1 image |
| 2026-10-26 | S1-U06 | `<609dd97>` | Saad | PASS | `logs/2026-10-26_S1-U04-U07.txt` | TEST=1 image |
| 2026-10-26 | S1-U07 | `<609dd97>` | Saad | PASS | `logs/2026-10-26_S1-U04-U07.txt` | TEST=1 image; U01/U02 also re-pass (regression) |
| 2026-10-27 | S1-U08 | `<ea7c7fc>` | Saad | PASS | `logs/2026-10-27_S1-U08-U10.txt` | TEST=1 image |
| 2026-10-27 | S1-U09 | `<ea7c7fc>` | Saad | PASS | `logs/2026-10-27_S1-U08-U10.txt` | TEST=1 image |
| 2026-10-27 | S1-U10 | `<ea7c7fc>` | Saad | PASS | `logs/2026-10-27_S1-U08-U10.txt` | TEST=1 image; U01–U07 also re-pass (regression) |
| 2026-10-27 | S1-U11 | `<2be9391>` | Saad | PASS | `logs/2026-10-27_S1-U11-U16.txt` | TEST=1 image |
| 2026-10-27 | S1-U12 | `<2be9391>` | Saad | PASS | `logs/2026-10-27_S1-U11-U16.txt` | TEST=1 image |
| 2026-10-27 | S1-U13 | `<2be9391>` | Saad | PASS | `logs/2026-10-27_S1-U11-U16.txt` | TEST=1 image |
| 2026-10-27 | S1-U14 | `<2be9391>` | Saad | PASS | `logs/2026-10-27_S1-U11-U16.txt` | TEST=1 image |
| 2026-10-27 | S1-U15 | `<2be9391>` | Saad | PASS | `logs/2026-10-27_S1-U11-U16.txt` | TEST=1 image |
| 2026-10-27 | S1-U16 | `<2be9391>` | Saad | PASS | `logs/2026-10-27_S1-U11-U16.txt` | TEST=1 image; U01–U10 re-passed old tests, U04–U07 with outage frames cut ~ 32 -> 15 B |
| 2026-10-28 | S1-U17 | `<9900c3b>` | Saad | PASS | `logs/2026-10-28_S1-U17a.txt`, `logs/2026-10-28_S1-U17b.png` | Phone hotspot, broker 172.20.10.3, Pico 172.20.10.7. hb 0–6 seen on laptop; "hello" and "hello2" received by Pico; s1 alive ticks 20–260 exactly 1000 ms apart through WiFi join |
| 2026-10-28 | S1-U18 | `<f071606>` | Saad | PASS | `logs/2026-10-28_S1-U18-U22.txt` | TEST=1 image |
| 2026-10-28 | S1-U19 | `<f071606>` | Saad | PASS | `logs/2026-10-28_S1-U18-U22.txt` | TEST=1 image |
| 2026-10-28 | S1-U20 | `<f071606>` | Saad | PASS | `logs/2026-10-28_S1-U18-U22.txt` | TEST=1 image |
| 2026-10-28 | S1-U21 | `<f071606>` | Saad | PASS | `logs/2026-10-28_S1-U18-U22.txt` | TEST=1 image |
| 2026-10-28 | S1-U22 | `<f071606>` | Saad | PASS | `logs/2026-10-28_S1-U18-U22.txt` | TEST=1 image; U01–U16 re-pass (regression) |