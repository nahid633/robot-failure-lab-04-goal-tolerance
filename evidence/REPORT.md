# Robot Failure Lab 04: one configuration change, two definitions of success

Measured Gazebo/Nav2 simulation. One route, six main trials in three pairs; verification runs are excluded. Standalone runner.

## Every trial

| Block | Configuration | Nav2 result | Time (sim s) | World error at result (m) | Settled max error (m) | Fixed 30 cm position gate | Recoveries |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 25 cm tolerance | succeeded | 15.354 | 0.253 | 0.180 | pass | 0 |
| 1 | 75 cm tolerance | succeeded | 14.403 | 0.742 | 0.615 | fail | 0 |
| 2 | 75 cm tolerance | succeeded | 14.052 | 0.739 | 0.616 | fail | 0 |
| 2 | 25 cm tolerance | succeeded | 18.303 | 0.241 | 0.187 | pass | 0 |
| 3 | 25 cm tolerance | succeeded | 55.113 | 0.226 | 0.171 | pass | 5 |
| 3 | 75 cm tolerance | succeeded | 14.103 | 0.746 | 0.622 | fail | 0 |

## Summary

- 25 cm tolerance: Nav2 success 3/3; fixed position gate 3/3; goal-to-result times 15.354, 18.303, 55.113 sim s.
- 75 cm tolerance: Nav2 success 3/3; fixed position gate 0/3; goal-to-result times 14.403, 14.052, 14.103 sim s.

## Within-seed comparisons

- Block 1: relaxed minus baseline time -0.951 sim s; settled error +0.436 m. Timing is secondary; failed actions are not completed journeys.
- Block 2: relaxed minus baseline time -4.251 sim s; settled error +0.429 m. Timing is secondary; failed actions are not completed journeys.
- Block 3: relaxed minus baseline time -41.010 sim s; settled error +0.450 m. Timing is secondary; failed actions are not completed journeys.

## Fixed method and limits

- Only navigation YAML difference: general_goal_checker.xy_goal_tolerance, 0.25 to 0.75 m. Live parameter snapshots before/after and across all trials verify exactly that difference.
- Original route (-2,-0.5) to (1,0.5), yaw zero; fresh processes and equal 1.5-sim-second pre-goal wait. No sensor faults, replacement goals, disabled monitor or changed speed setting.
- Position requirement was fixed before main runs: Nav2 success AND every world-pose sample 1.0–1.5 sim seconds after the result within 0.30m of target. This is an illustrative XY requirement, not a customer specification or docking test.
- World pose comes from a common observer-only Gazebo PosePublisher plugin. It is not derived from AMCL or wheel odometry and is never fed into navigation TF. The supplied sandbox map/world correspondence is assumed; spawn XY is checked within 0.05m. Model/base frames have equal XY origins.
- Terminal error uses the last acquired world pose at/before the action result, at most 0.15s old. Settled error uses a separately checked post-result window. Latest feedback is not substituted for ground truth.
- Seeds 101/102/103 are matched within pairs; AMCL/MPPI/scheduling remain uncontrolled. Three pairs on one route do not prove a population speedup, general reliability or physical performance. Null/mixed timing remains visible.
- This deliberately relaxes a documented acceptance policy. A mismatch against our fixed requirement is not a Nav2 bug or a production incident. Task success alone cannot establish unchanged task quality.
- Fresh-machine reproduction, Vektor integration, external use and public posting remain unverified. Preserve all excluded checks and startup/process anomalies in the archive.

## Retained attempts

Two block-3 startups exceeded readiness before any goal submission; separate attempt-2 directories retain their successful continuations. Main domains were 194, 195 and 196. Three excluded verification outcomes and the original recorder revision are retained under excluded-checks/. See ATTEMPT-AUDIT.json and FINDINGS.md; no completed task outcome was rerun.
