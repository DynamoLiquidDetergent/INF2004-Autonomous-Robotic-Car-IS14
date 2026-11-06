# Historical straight-drive test fixture

The overrides/ tree preserves the exact effective scratch-image files from the tested build, including temporary startup wiring. These are fixture snapshots under testing/, not edits to the production buddy directories. The build manifest records all tested working-source hashes, effective override hashes, flags and firmware identity. The three root policy/driver files are convenient copies of the same generated fixture.

To reproduce, use a separate copy of the matching tested source (verify the manifest hashes), apply these overrides at their repo-relative paths, and use the flags recorded in 2026-11-06_tested_build.json. The build used SMP=1, CONSOLE=usb_cdc, WIFI=none, NET=none, MQTT=0, TEST=0, CRASH_REPORT=0. Supply the team's Pico SDK/ARM toolchain as usual. Do not copy these fixtures over the team's production checkout or assume they enable line/barcode/network/obstacle tasks. Physical results still require an actual run.
