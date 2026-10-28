# Subsystem 1 — test log

**Subsystem:** Network Gatekeeper (WiFi, MQTT, telemetry)  ·  **Student ID:** 2502578  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Test IDs are never changed.

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
| S1-U23 | IF-13 (mbf_telem_status) and IF-15 (mbf_mqtt_tx, mbf_mqtt_rx) message buffers exist with the right max message sizes | IF-13, IF-15 | auto | make TEST=1, flash, read console | `[TEST] S1-U23 PASS` |
| S1-U24 | For ~2 s the test drains mbf_mqtt_tx (standing in for mqtt_utk.c): telemetry frames 500 ms apart, heartbeats 1000 ms apart (±10 ms), every frame unpacks | Telemetry 500 ms / heartbeat 1 s, IF-15 TX | auto | make TEST=1, flash, read console | `[TEST] S1-U24 PASS` |
| S1-U25 | An IF-13 status sent as S5 will send it appears in the next telemetry frame; new mission state sends STATE_CHANGE (new READY, old "not known") | IF-13 | auto | make TEST=1, flash, read console | `[TEST] S1-U25 PASS` |
| S1-U26 | Commands put in mbf_mqtt_rx as mqtt_utk.c will: PAUSE answered CMD_ACK accepted; code 7 and a 2-byte frame answered CMD_ACK rejected | FR7, FR8, IF-15 RX | auto | make TEST=1, flash, read console | `[TEST] S1-U26 PASS` |
| S1-U27 | Map put in mbf_mqtt_rx while READY: stored in IF-18, MAP_ACK accepted. After state leaves READY the same map gets MAP_ACK reason STATE | Prof mapping req., IF-18, IF-15 RX | auto | make TEST=1, flash, read console | `[TEST] S1-U27 PASS` |
| S1-U28 | Normal build with temporary fake S5: console shows `s1: tx telem` every 500 ms and `s1: tx hb` every 1000 ms for at least 10 s, and a STATE_CHANGE event at start and when the fake state flips at 10 s | Telemetry 500 ms / heartbeat 1 s, NFR4 | manual | make -j8 (normal image), flash, capture > 12 s | `t=` values step by 500 (telem) and 1000 (hb); 2 `tx evt` lines. *(Revised 2026-10-28: boot STATE_CHANGE happens before Serial monitor can start monitoring, so it can't be seen on the normal image. Covered by S1-U25.)* |
| S1-U29 | Link down 3 s, then up: nothing sent while down; kept frames come out marked replayed, in order, 1 per tick | IF-15, NFR4 | auto | make TEST=1, flash, read console | `[TEST] S1-U29 PASS` + `[INFO]` line |
| S1-U30 | Link down 12 s, then up: only newest 20 kept (oldest dropped); replayed in order, 1 per tick | IF-15 (20, drop-oldest) | auto | make TEST=1, flash, read console | `[TEST] S1-U30 PASS` + `[INFO]` line |
| S1-U31 | Real WiFi loss: hotspot off ~20 s, then on | IF-15, NFR4 | manual | build `WIFI=cyw43 NET=lwip`, flash, hotspot off/on, capture | `link down`, pub lines stop; `link up, 20 frame(s) to replay`, R frames 100 ms apart, then normal output; no reset |
| S1-U32 | Frames reach the broker over MQTT | IF-15 TX | manual | MQTT build, mosquitto_sub on is14/car/# | Telem 15 B every 500 ms and hb 4 B every 1 s. Hb shows WiFi and MQTT up |
| S1-U33 | Commands and maps from the PC | FR7, FR8, IF-15 RX, IF-18 | manual | mosquitto_pub on cmd and map | PAUSE accepted. Junk and too-long rejected. Map accepted when READY, rejected when not |
| S1-U34 | Broker stopped ~15 s, then restarted | IF-15, NFR4 | manual | Stop and restart Mosquitto | Link down with one retry line per attempt. Reconnects, then kept frames replay (header 41) ~100 ms apart |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Old rows are never deleted or edited to change their meaning / results.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-26 | S1-U01 | `c27b799` | Saad | PASS | `logs/2026-10-26_S1-U01-U02.txt` | TEST=1 image |
| 2026-10-26 | S1-U02 | `c27b799` | Saad | PASS | `logs/2026-10-26_S1-U01-U02.txt` | TEST=1 image |
| 2026-10-26 | S1-U03 | `c27b799` | Saad | PASS | `logs/2026-10-26_S1-U03.txt`, `logs/2026-10-26_S1-U01-U02.txt` | 1000 ms ±0 over 10+ ticks; also held during S2 timing tests |
| 2026-10-26 | S1-U04 | `609dd97` | Saad | PASS | `logs/2026-10-26_S1-U04-U07.txt` | TEST=1 image |
| 2026-10-26 | S1-U05 | `609dd97` | Saad | PASS | `logs/2026-10-26_S1-U04-U07.txt` | TEST=1 image |
| 2026-10-26 | S1-U06 | `609dd97` | Saad | PASS | `logs/2026-10-26_S1-U04-U07.txt` | TEST=1 image |
| 2026-10-26 | S1-U07 | `609dd97` | Saad | PASS | `logs/2026-10-26_S1-U04-U07.txt` | TEST=1 image; U01/U02 also re-pass (regression) |
| 2026-10-27 | S1-U08 | `ea7c7fc` | Saad | PASS | `logs/2026-10-27_S1-U08-U10.txt` | TEST=1 image |
| 2026-10-27 | S1-U09 | `ea7c7fc` | Saad | PASS | `logs/2026-10-27_S1-U08-U10.txt` | TEST=1 image |
| 2026-10-27 | S1-U10 | `ea7c7fc` | Saad | PASS | `logs/2026-10-27_S1-U08-U10.txt` | TEST=1 image; U01–U07 also re-pass (regression) |
| 2026-10-27 | S1-U11 | `2be9391` | Saad | PASS | `logs/2026-10-27_S1-U11-U16.txt` | TEST=1 image |
| 2026-10-27 | S1-U12 | `2be9391` | Saad | PASS | `logs/2026-10-27_S1-U11-U16.txt` | TEST=1 image |
| 2026-10-27 | S1-U13 | `2be9391` | Saad | PASS | `logs/2026-10-27_S1-U11-U16.txt` | TEST=1 image |
| 2026-10-27 | S1-U14 | `2be9391` | Saad | PASS | `logs/2026-10-27_S1-U11-U16.txt` | TEST=1 image |
| 2026-10-27 | S1-U15 | `2be9391` | Saad | PASS | `logs/2026-10-27_S1-U11-U16.txt` | TEST=1 image |
| 2026-10-27 | S1-U16 | `2be9391` | Saad | PASS | `logs/2026-10-27_S1-U11-U16.txt` | TEST=1 image; U01–U10 re-passed old tests, U04–U07 with outage frames cut ~ 32 -> 15 B |
| 2026-10-28 | S1-U17 | `9900c3b` | Saad | PASS | `logs/2026-10-28_S1-U17a.jpg`, `logs/2026-10-28_S1-U17b.txt` | Phone hotspot, broker 172.20.10.3, Pico 172.20.10.7. hb 0–6 seen on laptop; "hello" and "hello2" received by Pico; s1 alive ticks 20–260 exactly 1000 ms apart through WiFi join |
| 2026-10-28 | S1-U18 | `f071606` | Saad | PASS | `logs/2026-10-28_S1-U18-U22.txt` | TEST=1 image |
| 2026-10-28 | S1-U19 | `f071606` | Saad | PASS | `logs/2026-10-28_S1-U18-U22.txt` | TEST=1 image |
| 2026-10-28 | S1-U20 | `f071606` | Saad | PASS | `logs/2026-10-28_S1-U18-U22.txt` | TEST=1 image |
| 2026-10-28 | S1-U21 | `f071606` | Saad | PASS | `logs/2026-10-28_S1-U18-U22.txt` | TEST=1 image |
| 2026-10-28 | S1-U22 | `f071606` | Saad | PASS | `logs/2026-10-28_S1-U18-U22.txt` | TEST=1 image; U01–U16 re-pass (regression) |
| 2026-10-28 | S1-U23 | `df23e3a` | Saad | PASS | `logs/2026-10-28_S1-U23-U27.txt` | TEST=1 image |
| 2026-10-28 | S1-U24 | `df23e3a` | Saad | PASS | `logs/2026-10-28_S1-U23-U27.txt` | TEST=1 image |
| 2026-10-28 | S1-U25 | `df23e3a` | Saad | PASS | `logs/2026-10-28_S1-U23-U27.txt` | TEST=1 image |
| 2026-10-28 | S1-U26 | `df23e3a` | Saad | PASS | `logs/2026-10-28_S1-U23-U27.txt` | TEST=1 image; heartbeat after tests reports 3 rejected (matches U26/U27 inputs) |
| 2026-10-28 | S1-U27 | `df23e3a` | Saad | PASS | `logs/2026-10-28_S1-U23-U27.txt` | TEST=1 image; U01–U22 re-pass (regression) |
| 2026-10-28 | S1-U28 | `df23e3a` | Saad | PARTIAL PASS | `logs/2026-10-28_S1-U28.txt` | telem 500 ms / hb 1000 ms over 18 s. STATE_CHANGE at the 10 s flip captured. Issue: boot event happened before monitor opened (visible in U25) |
| 2026-10-28 | S1-U28 | `36f0330` | Saad | PASS | `logs/2026-10-28_S1-U28-rerun.txt` | Repeat successful results. STATE_CHANGE at all 4 flips. This test was run AFTER S1-U28's expected result was revised. |
| 2026-10-28 | S1-U29 | `1ee9edc` | Saad | PASS | `logs/2026-10-28_S1-U29-U30.txt` | 5 replayed, 100 ms apart, in order |
| 2026-10-28 | S1-U30 | `1ee9edc` | Saad | PASS | `logs/2026-10-28_S1-U29-U30.txt` | 20 replayed, 100–103 dropped, 100 ms apart, in order; U01–U27 re-pass |
| 2026-10-28 | S1-U31 | `1ee9edc` | Saad | PASS | `logs/2026-10-28_S1-U31.txt` | Off ~20 s; 20 kept, oldest dropped. on rejoin 20 kept (hb outage=20), oldest dropped, replayed 1 per 100 ms in order, 5 live frames queued behind. no reset |
| 2026-10-29 | S1-U32 | `235f63e` | Saad | PASS | `logs/2026-10-29_S1-U32.txt` | Telem 500 ms, hb 1 s. 12 boot frames replayed after connect |
| 2026-10-29 | S1-U33 | `235f63e` | Saad | PASS | `logs/2026-10-29_S1-U33.txt` | All 5 ACKs as expected. Hb reject count 0→3. Map bit kept after rejected map |
| 2026-10-29 | S1-U34 | `235f63e` | Saad | PASS | `logs/2026-10-29_S1-U34.txt` | Broker off 15 s, reconnect 1 s after restart. 25 replayed in order, hb outage 18→0 |