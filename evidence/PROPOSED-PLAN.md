---
type: experiment
status: active
updated: 2026-10-18
tags: [vektor, robot-failure-lab, simulation, nav2, configuration, regression]
sources: [robot-failure-lab-next-experiments-2026-10, robot-failure-lab, robot-failure-lab-03-localization-feed, robotics-attention-playbook-2026-09]
confidence: medium
---

# Robot Failure Lab 04: one configuration change, unexpected regression

## Selection and question

On October 18 the founder selected the configuration-comparison proposal: “lets dp this One configuration change, unexpected regression.” Local implementation, simulation, verification and a sharing package are authorized. Public posting and outbound communication are not part of this execution.

The assistant selected one documented acceptance-policy change: `controller_server.general_goal_checker.xy_goal_tolerance`, from 0.25 m to 0.75 m. Question: does a relaxed arrival tolerance make the original navigation task report completion sooner while worsening position against an unchanged task requirement? A tradeoff is a hypothesis, not a result guaranteed in advance. This is a deliberately constructed configuration example, not a discovered production incident or a Nav2 defect.

## Protocol fixed before measured trials

- Same installed ROS 2 Jazzy/Nav2/Gazebo TurtleBot3 sandbox as prior labs; fresh simulator and navigation processes for every trial. Start (-2,-0.5), target (1,0.5), yaw zero. No scan delay, blackout, replacement goal or changed speed setting.
- Baseline XY tolerance 0.25 m; candidate 0.75 m. Recursively verify exactly one leaf differs in navigation YAML. Capture live parameters from controller, AMCL, planner, collision monitor, velocity smoother and both costmaps before and after every trial. Require no within-trial parameter changes and check the between-condition runtime difference.
- Fixed illustrative task-position requirement: original goal succeeds AND every independently measured world-position sample between 1.0 and 1.5 simulated seconds after the result lies within 0.30 m of the target. This requirement is selected for this demonstration; it is not a supplied customer requirement, physical docking criterion or orientation test.
- Primary outcomes: original action result, settled maximum world-position error and pass/fail against the fixed requirement. Secondary: action-result time, world/localized position error at result, world path length, recoveries and world yaw error. Never compare a failed action's duration as a successful journey.
- Common observer instrumentation adds Gazebo PosePublisher to a retained copy of the installed robot model. Bridge its model pose only onto `/rfl04/truth`; never inject truth into `/tf`. Record world pose separately from localization and odometry. In the inspected model, model/base_footprint/base_link have equal XY origins. Check initial world position within 0.05 m of spawn. Use supplied sandbox map/world coordinates without fitting an alignment.
- Terminal error uses the last pose acquired at/before result, at most 0.15 s old. Settled window needs >=8 samples, endpoint coverage and <=0.15 s gaps. Record acquisition and receipt timestamps. Preserve action terminal and post-result observation times separately.
- Two separately retained verification trials (baseline/candidate, seed 100) before main execution. If instrumentation fails, retain attempts and fix before main; do not mix them into the measured sample.
- Six main trials: seed 101 baseline/candidate, seed 102 candidate/baseline, seed 103 baseline/candidate. Equal 1.5 simulated-second pre-goal wait after readiness and parameter inspection. Match Gazebo seeds within pairs; AMCL/MPPI and scheduling remain uncontrolled. No concurrent experiment workload.
- Bound readiness to 75 wall seconds after worker start, task to 90 simulated / 270 wall seconds, and post-result observation to 1.8 simulated seconds with a 20-wall-second watchdog. Retain every attempt. Only before-goal infrastructure errors may be continued in a new attempt directory, up to three starts per scheduled trial. Never rerun a completed task outcome.
- Freeze source, model, map, configuration and package snapshots. Require main source/config/packages to match verified checks. Do not change the requirement, select a favorable seed, extend the sample or switch parameters after seeing measured results. Null or mixed results remain visible.

Three pairs on one route are exploratory. Timing variability already occurred in earlier labs. A shorter reported action time is useful only alongside unchanged task-quality acceptance. This standalone experiment does not evaluate Vektor, establish physical performance or show external adoption.

## Implementation and technical basis

Separate implementation: `code/robot-failure-lab/experiment04/`. Prior code and evidence remain intact. Six focused tests check one-leaf/no-mutation comparison, success-versus-task-quality mismatch, failed-action exclusion, missing/stale/sparse evidence, terminal-versus-post-result separation and close-position acceptance. They passed before the first verification launch. Python compilation passed.

Primary references retrieved October 18, 2026:

- [Nav2 Jazzy SimpleGoalChecker configuration](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/controller_server/controller_server_plugins/simple_goal_checker/): XY tolerance is part of configured goal completion.
- [Jazzy goal-checker source](https://api.nav2.org/nav2-jazzy/html/simple__goal__checker_8cpp_source.html): stateful XY/yaw acceptance logic. The configured change is intentional; independent task acceptance must remain fixed.
- [Gazebo 8 PosePublisher](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1PosePublisher.html): model-pose publishing and vector messages. Installed plugin behavior still requires empirical verification before main measurements.

## Execution status and next action

Two excluded verification trials launched under `/tmp/robot-failure-lab-04-verification-20261018` in WSL, ROS domain 193. At this entry no verified outcomes or main results are recorded. Next: validate instrumentation and parameter isolation, then execute the frozen six-run sample, report every result and prepare a local sharing package with an unsent explanation.

Related: [[robot-failure-lab-next-experiments-2026-10]], [[robot-failure-lab]], [[robot-failure-lab-03-localization-feed]], [[vektor]], [[robotics-attention-playbook-2026-09]].
