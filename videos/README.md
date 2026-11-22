# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the LED blinking, the motor turning, the robot reaching
the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds. One test or one feature per video.
- **Say what it is.** Show or say the test ID (e.g. `S2-U03`, `INT-01`) or
  the demo name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment, or put it in the table below.
- **Show the result.** Keep both the hardware and, if it matters, the
  console output in view.
- **Landscape**, steady, good light. No music needed.

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the table below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's row in
`testing/.../TEST_LOG.md`.

## Video index

Newest at the bottom. Never delete a row.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1 | S1-U03 | `abc1234` | | <https://youtu.be/...> | LED blinks at 2 Hz |
| 2026-10-29 | S3 | S3-I01 (printed `INT-01`) | `494b82f` | 2500172 | https://youtu.be/egCM2ayKMbE | `line_sensor_task` delivers `LINE_DATA_MSG` on `q_line_data` within 100 ms; calibration timed out (left/centre sensors unplugged), default range kept |
| 2026-10-30 | S3 | S3-M03 | `ccf825a` | 2500172 | https://youtu.be/FmM1FnGhDMA | robot drives at 100 mm/s over `*Z*` then `*A*`; each decoded once (`ch=Z cmd=2`, `ch=A cmd=1`) without stopping; Z-then-A run only |
| 2026-11-04 | S3 | S3-M04 line-following demo | `90e353c` | 2500172 | https://youtube.com/shorts/brAS69HngTo | follows the black line on its own with the left + centre sensors, through curves (S3 demo standing in for S5's Vehicle Controller) |
| 2026-11-22 | S5 | S5-M33 run 22 (also SYS-06, SYS-07, SYS-08) | `737713a` | 2503437 | https://youtu.be/LF7pgxlZWmA | 10 cm box about 6 cm right of the line: car stops at 30 cm, scans, creeps and scans again, goes round the left and back on the line; dashboard shows the obstacle width and distance |
| 2026-11-22 | S5 + S1 | S5-M43 run 1 (also SYS-09) | `737713a` | 2503437 | https://youtu.be/0yQvYQC1reo | RESUME from the dashboard: car drives; hotspot off: RESUME does not reach the car, it stays stopped with no fault; hotspot back on: link up, buffered telemetry replayed, RESUME drives again |
| 2026-11-22 | S4 + S5 + S1 | S4-H10 trial 4 (also SYS-03) | `737713a` | 2503437 | https://youtu.be/OXtJbN9jTqQ | integrated car line following over the 23 mm reference hump; S4 measures it and the dashboard shows the highest hump height over MQTT (23 mm) |
| 2026-11-22 | S5 + S3 + S1 | S5-M42 run 10 (also SYS-01, SYS-02, SYS-05) | `9240c6c` | 2503437 | https://youtu.be/ki35ANcpQ7Y | map Forward B, U-turn A, Left C, Right D sent from the dashboard while READY (Map OK); RESUME; barcode A read and decoded as U-TURN; car turns about 180 degrees at the junction and follows the line back |
| 2026-11-22 | S5 + S3 + S1 | S5-M42 run 8 (also SYS-01, SYS-02, SYS-05) | `9240c6c` | 2503437 | https://youtu.be/kiAJ1WYLtMQ | map sent from the dashboard while READY with A = Left (Map OK); RESUME; barcode A read and decoded as LEFT; car turns left at the junction; something seen ahead after the turn, car stops to scan and is paused from the dashboard |
| 2026-11-21 | S5 + S4 | S5-M40 run 1 (also SYS-10) | `d6249b1` | 2503437 | https://youtu.be/ZhrfBhRf2VI | car lifted while line following; console prints `handling: PICKED UP` and `vc: PICKED UP: motors stopped, run over`; wheels stop and stay stopped when it is put down (end of the demo) |
