# Subsystem 4 — latest right-wheel hump tests

Owner: Bryan (`xcher21`), student ID 2500767. Group IS14.

## Scope and method

Three powered line-following traversals of the operator-measured 23 mm hump, 2026-11-07 (Asia/Singapore). Right large tyre crosses; left tyre stays on the same flat floor. Measured wheel spacing: 120 mm. The caster may move. Maximum test-image line speed: 50 mm/s.

The table records the console-displayed heights after a fixed 5 mm display correction. This correction is outside the firmware; its stored measurements are unchanged. Original captures are retained verbatim. These results validate this three-run display experiment, not uncorrected firmware accuracy or calibration across other hump heights.

## Results

| Trial | Completion (SGT) | Displayed height (mm) | Error against 23 mm | Within ±10% | Evidence |
|---|---|---|---|---|---|
| 1 | 22:30:40 | 24 | 4.3% | PASS (displayed value) | [Capture](logs/2026-11-07_223022_054533_right_wheel_hump_trial1.txt) |
| 2 | 22:31:54 | 24 | 4.3% | PASS (displayed value) | [Capture](logs/2026-11-07_223135_967778_right_wheel_hump_trial2.txt) |
| 3 | 22:32:57 | 23 | 0.0% | PASS (displayed value) | [Capture](logs/2026-11-07_223235_721306_right_wheel_hump_trial3.txt) |

All three runs completed LEVEL → CLIMBING → DESCENDING → LEVEL, recorded exactly one hump, and retained a highest value. Displayed results span 1 mm. No reported line loss, reverse motor motion, Safety fault, read error, impact, aborted traversal or missed release occurred in these captures. Short missing-sample intervals are disclosed in the captures.

## Traceability

Tested working-tree base: `2d287d78ec02a107c0f16f6a31a3b4e557c78bef`; build tag `WT-609dab8d2247`. These identifiers alone do not identify the test-image overrides.

Tested firmware SHA-256: `d240390f73fb1fcc6731a151acd96dd408eb0985afd0c0d2c0dc6844a30c865f`.

Console runner SHA-256: `e9f9f26f4e0237d4ed653eb4cf49c9c6989fda1555974b87619af29d7eced063`.
Viewer SHA-256: `fa0c130eeb777552a32f9c24cf0a1461d24d576d51f42473d4ca392651cd34a3`.

S4-H12 functional completion: PASS for these three captures. S4-H14 fixed display-correction experiment: 3/3 within ±10% for this 23 mm hump. S4-H10 uncorrected physical height accuracy is not passed by this display correction. S4-H13 maximum selection across different-height humps in one boot: NOT RUN.

## Code and evidence limits

The checked-in preparation contains the tested S4 algorithms and portable regression tests. Test-image startup helpers/build stamps are removed; the existing repository startup, S2/S3/S5 code, GPIO assignments and line/crossing configuration are preserved. The full repository firmware assembled with those existing settings has NOT been retested after this S4-only preparation. Do not identify a later commit as the firmware tested above.

Only the latest three captures are kept in this folder at Bryan's request. The previous TEST_LOG and removed development captures are backed up outside the repository by the installer; their observations are not changed or reclassified.

Before deployment: verify the normal repository build/integration; assess other hump heights and support/contact conditions; investigate previous duplicate detections; measure complete timing/stack/RAM limits. The display experiment does not establish a general sensor calibration or prove the duplicate-detection issue is fixed.

Prepared-source software verification: 16 portable C suites PASS; 48 ARM application units passed syntax checking. The normal GNU build attempt in the preparation environment stopped when Windows make raised exception `0xc0000008`; a complete link has not been verified here. Run the normal repository build before describing this preparation as build-verified.
