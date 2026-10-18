# Robot Failure Lab 04: faster completion, worse arrival?

Standalone ROS 2 Jazzy / Nav2 / Gazebo TurtleBot3 configuration comparison.
Exactly one YAML leaf changes: controller_server general_goal_checker
xy_goal_tolerance, 0.25 to 0.75 m. No sensor faults. Collision monitoring and
all other navigation settings remain unchanged. Two excluded verification
trials precede six fresh-process main trials, alternating condition order by
Gazebo seed 101/102/103. AMCL/MPPI randomness and scheduling are uncontrolled.

The independent illustrative position requirement is fixed at <=0.30 m from
the target in every Gazebo model-world-pose sample between 1.0 and 1.5 simulated
seconds after the original action terminates, with an action success required.
This measures position only: no docking, contact, physical safety, statistical
reliability or Vektor integration claim. Retain null/mixed results honestly.

Common observer instrumentation adds PosePublisher to a COPY of the installed
robot SDF and bridges only to /rfl04/truth, never /tf. The supplied sandbox's
map/world XY correspondence is used without fitting. Model/base_footprint/
base_link share XY origin in the inspected model. Pre-goal world XY must agree
with spawn within 0.05 m. Terminal and settled world errors are separate from
the map-localized estimate and Nav2's configured acceptance.

Supported environment: Ubuntu/WSL, ROS 2 Jazzy with nav2_bringup,
nav2_minimal_tb3_sim, Gazebo 8 vendor plugins, ros_gz_bridge, Python3, PyYAML,
matplotlib, NumPy and Pillow. Inspect packages.txt for exact measured versions.

```bash
source /opt/ros/jazzy/setup.bash
python3 -m unittest -v test_core
python3 run.py /tmp/rfl04-verification --verification --domain 193
python3 run.py /tmp/rfl04-main --verification-root /tmp/rfl04-verification --domain 193
python3 report.py /tmp/rfl04-main
```

Use new output directories and an idle domain; no concurrent simulation workload.
Snapshot code/configuration before execution. Main execution requires the same
verified source/config/package hashes. Every original action outcome is retained.
Startup bounded to 75 wall seconds after worker launch; task bounded to 90 sim
or 270 wall seconds; 1.8 sim-second passive observation follows the result.
No replacement goals. --resume-infrastructure only continues before-goal startup
failures, preserving all old directories/results with a logged continuation and
up to three starts per scheduled trial. A task result is never retried.

Each trial includes live parameter snapshots before/after, world and localized
poses, scans, odometry, commands, feedback, results and logs. validate.py recomputes
metrics and checks telemetry freshness, configuration and isolated truth routing.
Fresh-machine reproduction remains unverified until an independent setup test.

Technical references retrieved 2026-10-18:
- https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/controller_server/controller_server_plugins/simple_goal_checker/
- https://api.nav2.org/nav2-jazzy/html/simple__goal__checker_8cpp_source.html
- https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1PosePublisher.html

Interpretation: this deliberately tests a documented acceptance-policy change.
Any apparent regression is relative to our unchanged task-position requirement,
not evidence of a Nav2 defect or an accidentally discovered production incident.
