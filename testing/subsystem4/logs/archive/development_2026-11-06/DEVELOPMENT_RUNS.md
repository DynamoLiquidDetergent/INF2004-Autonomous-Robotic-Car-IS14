# Superseded development attempts

Excluded from the later 50-mm/s validation set because they used earlier configurations. Original results are preserved; they are not reclassified as operator mistakes or changed to PASS.

### S4-only powered height run 2026-11-06_143942_669802

| Date | Test ID | Commit | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-06 | S4-H10 | `a8a917c` | Bryan | INCOMPLETE | [Summary](2026-11-06_143942_669802_s4_height_only_summary.json); [Fixture](2026-11-06_143942_669802_s4_height_only_fixture.py) | S4 height only, straight powered drive and real encoders; one fresh boot per trial. ['FAIL']. Hardware timer and kernel time disagree. Height is correctly blocked; timing setup still needs fixing. fault_rc=0 |


### S4-only powered height run 2026-11-06_144038_313966

| Date | Test ID | Commit | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-06 | S4-H10 | `a8a917c` | Bryan | INCOMPLETE | [Summary](2026-11-06_144038_313966_s4_height_only_summary.json); [Fixture](2026-11-06_144038_313966_s4_height_only_fixture.py) | S4 height only, straight powered drive and real encoders; one fresh boot per trial. ['FAIL']. Hardware timer and kernel time disagree. Height is correctly blocked; timing setup still needs fixing. fault_rc=0 |


### Motor-powered distance run 2026-11-06_144728_207968

| Date | Test ID | Commit | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-06 | S4-H11 | `a8a917c` | Bryan | FAIL | [Summary](2026-11-06_144728_207968_powered_distance_summary.json) | Real S5 line-following, one fresh boot per trial. ['FAIL']. Hardware timer and kernel time disagree. Height is correctly blocked; timing setup still needs fixing. fault_rc=0 |


### Motor-powered distance run 2026-11-06_144748_300174

| Date | Test ID | Commit | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-06 | S4-H11 | `a8a917c` | Bryan | FAIL | [Summary](2026-11-06_144748_300174_powered_distance_summary.json) | Real S5 line-following, one fresh boot per trial. ['FAIL']. Hardware timer and kernel time disagree. Height is correctly blocked; timing setup still needs fixing. fault_rc=0 |
