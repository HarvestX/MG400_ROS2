# MG400 ServoJ RViz panel

The `mg400_rviz_plugin/ServoJController` panel provides Enable/Disable,
explicit ServoJ session Start/Stop, four target sliders, and status displays.
Add it in RViz via **Panels > Add New Panel**. The default
`mg400_bringup/rviz/mg400.rviz` does not include this panel; save the RViz
configuration after adding it if it should reopen automatically.

The panel uses fixed `/mg400/robot_state`, `/mg400/joint_states`,
`/mg400/enable_robot`, `/mg400/disable_robot`, `/mg400/servo_j_session`, and
`/mg400/servo_j` names. Use ROS remapping for a different robot namespace.
ROS callbacks and topic publication run in a separate executor thread. The
ServoJ publisher uses best-effort, volatile, depth-one QoS. The frequency
control sets topic publication to 10-33 Hz (default 30 Hz); RViz saves it.
It can be changed only while the panel session is idle. The driver's 30 ms
send timer and synchronous TCP transport do not guarantee the same rate at
the robot.

The status area shows robot state, raw mode, feedback freshness and ages;
session phase and ID; requested and measured topic publish rates, last point
age, and limit setting. The detail line reports the latest operation or error.
The measured rate counts topic publications, not executed robot commands.

Enable is available only when fresh feedback reports `DISABLED` and the
panel session is idle. Wait for `ENABLED` feedback before pressing Start.
Disable requests `DisableRobot` and stops ServoJ publication first; use
the robot status, not just the service response, to confirm the transition.

The driver publishes eight visualization joints. The panel maps native ServoJ
J1-J4 from joint names `mg400_j1`, `mg400_j2_1`, `mg400_j4_2`, and
`mg400_j5`, respectively; J3 is the controller's absolute angle. Prefixes and
message order are supported. Outside a session, sliders follow fresh feedback.
During a session, sliders represent targets and feedback is shown separately.
Click a slider to focus it, then use Left/Right to change its target by
0.1 degree per key press. Sliders remain disabled outside an active session.
The focused slider has a blue outline and its joint label shows `[KEY]`;
both indicators clear when keyboard focus moves elsewhere.

Start requires `ENABLED` robot state received within 500 ms, valid joint
feedback received within 200 ms, and a target inside MG400 joint limits. After
the session service succeeds, the first point uses the latest feedback
angles at full precision, not the rounded slider values. The
speed/acceleration limit checkbox starts checked and can be changed only
while the session is idle. With it checked, subsequent points are limited
to 10 degrees/s and 50 degrees/s^2 per joint. Without it, slider targets
are published directly and may cause abrupt motion. Invalid target
combinations hold the previous command; an invalid interpolated point
stops streaming. The driver checks joint limits in either mode.

Stop immediately ceases publication and retries session release while the
driver waits at least 300 ms after its last sent point and for fresh raw
`ENABLE` feedback. Disable also ceases publication before requesting
`DisableRobot`. Stale state or joint feedback, or a publish gap over 120 ms,
stops the stream and triggers a stop attempt. Closing the panel makes a
bounded best-effort release request; do not assume the session was released
if the robot cannot satisfy the stop conditions.

The panel cannot confirm that MG400 accepted a ServoJ command: its publish
rate reports ROS topic sends, and Motion TCP does not read command replies.
Verify motion and timing on the robot. Keep the physical emergency stop
available during first tests. Start near a safe central pose, confirm that
pressing Start alone causes no movement, then move one slider by a small
amount before testing Stop and Disable.
