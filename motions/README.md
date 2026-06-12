# Motion Library

Visually-validated commanded movements for the Go2 sim, driven over ros-mcp
(rosbridge → `/sim/cmd_vel` and `/sim/joint_cmd`).

Each motion is one JSON file replayable verbatim with ros-mcp
`publish_for_durations(topic, msg_type, messages, durations, rate_hz)`.

**Validation protocol:** Claude announces the motion, streams it to the live
sim, and Ethan watches the stream and rules PASS or FAIL. Only PASSED motions
are committed with `"status": "validated"`. Failed attempts are either tuned
and re-run or recorded as `"status": "rejected"` with notes.

Joint convention: `sensor_msgs/JointState`, URDF names (`FL_thigh_joint` …),
radians, partial joint sets allowed (unnamed joints hold). DOF order in the
sim is type-grouped (hips×4, thighs×4, calves×4 — Genesis breadth-first).
Defaults: hips 0, FL/FR thigh 0.8, RL/RR thigh 1.0, calves −1.6.
Velocity convention: `geometry_msgs/Twist`, normalized stick units [−1, 1].

Safety: DIRECT_JOINT mode suspends fall-over termination while the joint
stream is fresh (<0.5 s); operator gamepad input or ESTOP aborts instantly.
