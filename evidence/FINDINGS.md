# One YAML change. Same reported success rate. Different arrival quality.

Measured October 18, 2026: six planned main Gazebo/Nav2 trials, three per configuration, on one route.

| Configuration | Nav2 success | Fixed 30 cm position requirement | Settled world error range |
| --- | --- | --- | --- |
| 25 cm tolerance | 3/3 | 3/3 | 17.1–18.7 cm |
| 75 cm tolerance | 3/3 | 0/3 | 61.5–62.2 cm |

The only navigation setting changed was general_goal_checker.xy_goal_tolerance: 0.25 to 0.75 m. All six original actions reported success. The independent illustrative arrival requirement stayed fixed: every measured world-position sample 1.0–1.5 sim seconds after success within 0.30 m of the unchanged target.

## What the evidence means

The relaxed configuration reported success while failing our unchanged position requirement in all three measured trials. This is a task-quality regression relative to that requirement. It deliberately changes a documented acceptance policy; it does not expose a Nav2 software defect or establish an accidental production incident.

See REPORT.md for every runtime and endpoint, including per-seed differences and recoveries. Time is secondary. Three pairs on one static route, with uncontrolled AMCL/MPPI randomness and scheduling, do not establish a general speed improvement. The excluded revised verification candidate took 54.660 sim seconds, reinforcing the timing limitation.

The practical release-testing lesson: keep application acceptance criteria independent of the navigation configuration being evaluated. An unchanged success rate can hide changed arrival quality. The 30 cm example is our predeclared demonstration requirement, not a customer-supplied docking specification.

## Measurement and configuration checks

Gazebo publishes the simulated model pose through common observer-only instrumentation. That stream is never fed into navigation TF. World endpoint errors are separate from AMCL/localized errors and wheel odometry. Model/base frames share XY origins in the inspected model; initial world position agrees with spawn within 0.05 m. The supplied sandbox map/world alignment is assumed, not fitted.

Each trial retained live snapshots of seven navigation nodes before and after. All six trial snapshots were stable; between configurations, exactly one live parameter differs. All six records passed independently recomputed metric, pose age/coverage, scan continuity, truth-stream isolation, task-result and source-hash checks. Six focused core tests passed before execution.

## Every attempted run is retained

Three excluded verification outcomes are preserved: the first baseline exposed a parameter-recorder issue; two revised verification trials passed. An uninitialized collision-monitor declaration made the original bulk query return no values. Revised capture queries individual declarations and marks unavailable values explicitly. This was fixed before main execution; original evidence was retained.

Two main block-3 launches (baseline and relaxed) exceeded the 75-wall-second readiness bound before READY or any goal submission. Their logs remain at block-3-baseline/ and block-3-relaxed/. Bounded continuations used fresh ROS domains and preserved respectively four and five completed outcomes. Successful startup continuations have separate attempt-2 directories. No completed task outcome was rerun. Main domains: 194, 195 and 196. The relaxed startup log contains a failed controller lifecycle-service response; underlying startup causes were not established.

ATTEMPT-AUDIT.json retains all process-death lines and shutdown ordering. Recorded exit-code-11 events across all attempts: 0. Their causes are not inferred from exit codes.

## Package and limits

REPORT.md, comparison.png, first-pair-paths.png and first-pair-recorded.gif show the measured evidence. The animation reconstructs the predeclared first pair from recorded world poses over the supplied map; it is not a screen recording or physical footage. Faster action completion is not assumed to mean better task performance.

PROTOCOL.json, PROPOSED-PLAN.md, source/, exact configuration/map/package snapshots, per-trial logs/CSVs, validation files, the retained failed startup and excluded-checks/ preserve provenance. INPUT-MANIFEST.json explicitly records capture after the first main outcome; the main protocol/source/config snapshots preceded execution.

reproduction/ contains standalone source without private-wiki dependencies. Fresh-machine reproduction remains unverified. No physical robot, collision/docking validation, Vektor integration, external use, public upload or outbound message is established.
