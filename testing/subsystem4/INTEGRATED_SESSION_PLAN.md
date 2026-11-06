# Integrated session plan — 2026-11-07

Status: NOT RUN. Keep these planned cases separate from the 2026-11-06 straight-height results. Assign the next available shared INT identifiers with the relevant owners in testing/integration/TEST_LOG.md before executing; this draft does not allocate or edit another buddy's IDs/resources.

| Proposed check | Setup and procedure | Expected observation / acceptance | Status |
|---|---|---|---|
| Startup and calibration | Build/flash the intended integrated profile; reset flat and still; record exact commit plus overrides | IMU calibrated, clock ready, fresh odometry before movement; no unexplained startup fault | NOT RUN |
| S3 line reading with S2 forward movement | Use a continuous known line; observe centred/left/right/lost readings and both wheel speeds | Actual line reports change correctly and controller commands the agreed steering; no fabricated ADC values | NOT RUN |
| Forward distance readiness | Independently measure a flat path; compare start/end cumulative odometry and measured distance | Apply existing S4-H11 <=5% target; no false hump on flat ground | NOT RUN |
| Line following across hump with S4 height | Align approach/hump/exit line; traverse 23-mm reference hump without handling robot; repeat three times | Full terrain sequence, real height and highest, each error <=10%; record EVERY trial including failures | NOT RUN |
| Line loss behaviour | Confirm whether the build uses normal team recovery or the local bounded unlined-hump demo; test in the agreed setup | Behaviour matches the selected configuration. Do not claim the local override is production S5 integration | NOT RUN |
| Highest across multiple humps | Use independently measured different humps in a single boot | Stored maximum equals largest valid estimate, rather than merely echoing the last hump | NOT RUN |
| Stop/fault path | Coordinate an agreed safe fault injection with Safety owner; record original bits/stop commands | Existing Safety ownership retained; actual stop observed; no unmeasured stopping-distance/latency claim | NOT RUN |

Before running: confirm the axle-to-caster wheelbase measurement and mount; keep code/test fixture fixed during a series. The isolated HEIGHT_ONLY image does not start S3. A line-following build is a different image and requires new results. The local line demo disables networking, barcode and obstacle tasks; use the team's intended full build to test those features. Do not assume the successful 50-mm/s straight run transfers to the line controller's different speed/steering behaviour.

Save exact console captures and build identifiers. Use the physical switch for a stop: Ctrl+C only closes the monitor. Record observations and actual results after each case; no cases in this plan are pre-approved as PASS.
