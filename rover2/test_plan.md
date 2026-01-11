Rover1 Test Plan

File: test_plan.md
Last Updated: 2025-12-22
Purpose: A comprehensive, step-by-step verification plan for Rover1 hardware, drivers, sensor fusion, localization, and autonomy. This plan is designed for human execution with AI-coded software. Each test includes objective pass/fail criteria and “what to report to AI” so bugs can be fixed quickly.

⸻

0. Ground Rules

0.1 Safety Requirements (Non-Negotiable)
	•	Physical E-Stop is reachable at all times during any test that can move wheels.
	•	Rover is elevated on a stand for all motor/encoder tests until explicitly marked “GROUND TEST.”
	•	Battery is connected through a master switch; power-off must be instant.
	•	Never run autonomy outdoors until:
	•	Watchdog verified
	•	E-stop verified
	•	Motor enable/disable verified
	•	Localization verified

0.2 Test Philosophy
	•	Test one variable at a time.
	•	Prefer short, repeatable scripts over long manual sequences.
	•	Always capture:
	•	terminal logs
	•	ros2 topic echo snapshots
	•	ros2 node list, ros2 topic list
	•	Foxglove screenshots for key signals

0.3 Test Artifacts to Save

Create a folder per test day:
	•	logs/YYYY-MM-DD/
	•	terminal_<testname>.txt
	•	foxglove_<testname>.png
	•	ros2_<testname>.txt (node list, topic list, params)
	•	bags/<testname>/ (if recorded)

⸻

1. Environment & Workspace Verification

1.1 OS, ROS, Workspace

Goal: Confirm ROS 2 Jazzy + workspace are correct and consistent.

Steps
	1.	SSH into rover:

ssh andrewmeckley@rover1.local


	2.	Confirm ROS:

which ros2
ros2 --version
echo $ROS_DISTRO


	3.	Confirm workspace builds cleanly:

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash



Pass Criteria
	•	ROS_DISTRO = jazzy
	•	Build succeeds with no errors.

Report to AI if Fail
	•	Full build output.
	•	Any missing deps or import errors.

⸻

2. Repo Ops & Deployment Tests

2.1 Auto-Update Service Health

Goal: Confirm systemd auto-update does not break runtime.

Steps

systemctl status rover1-updater.service
systemctl status rover1-updater.timer
journalctl -u rover1-updater.service --no-pager -n 200

Pass Criteria
	•	Service runs without repeated failures.
	•	No unintended resets or permission errors.

Report to AI
	•	status output + last 200 journal lines.

⸻

3. Hardware Bus & Device Discovery

3.1 I2C Device Scan

Goal: Confirm motor HAT + IMU sensors show up reliably.

Steps

i2cdetect -y 1

Expected
	•	Motor HAT: 0x34
	•	IMU devices: 0x1c (LIS3MDL), 0x6a (LSM6DSL), 0x77 (BMP388)

Pass Criteria
	•	All addresses present, stable across reboots.

Report to AI
	•	Screenshot/text of i2cdetect.
	•	Any missing addresses and when they disappear.

⸻

4. Motor Driver + Watchdog (Bench)

Rover must be on a stand. Wheels free-spinning.

4.1 Motor Register Mapping Sanity

Goal: Confirm register mapping (51–54) matches wheel positions.

Steps
	1.	Run a very low speed per wheel (one at a time).
	2.	Use your existing probe tool:

python3 probe_motors.py 51 20
python3 probe_motors.py 52 20
python3 probe_motors.py 53 20
python3 probe_motors.py 54 20



Pass Criteria
	•	Each register spins the intended wheel (per your mapping).
	•	No other wheel spins.

Report to AI
	•	“Reg 51 spun Rear Left only” style mapping confirmation.
	•	Any mismatches (which wheel moved).

4.2 Direction Inversion Validation

Goal: Confirm “forward” command matches expected wheel rotation direction.

Steps
	•	For each wheel, command forward then reverse.
	•	Mark the wheel with tape and visually confirm.

Pass Criteria
	•	Forward command produces consistent forward rotation for that wheel in the robot frame.

Report to AI
	•	Which wheels are reversed vs expected.
	•	Any wheel that behaves inconsistently.

4.3 Watchdog Stop Test (0.5s)

Goal: Verify runaway prevention.

Steps
	1.	Start a wheel at moderate speed.
	2.	Stop sending commands.
	3.	Time how long until it stops.

Pass Criteria
	•	Motor stops within ≤ 0.6s after command stream stops.

Report to AI
	•	Measured stop time and method used.
	•	If it fails: does it keep going indefinitely or stop late?

4.4 Motor Enable/Disable Gate (Recommended)

Goal: Ensure motors cannot move unless explicitly enabled.

Steps
	•	If implemented: test service/topic that enables motors.
	•	Attempt wheel command when disabled.

Pass Criteria
	•	Commands ignored when motors disabled.

Report to AI
	•	How gating is implemented (param/service/topic) and behavior.

⸻

5. Encoder Verification (Bench)

Rover on stand. Move wheels by hand and with motor.

5.1 Raw Encoder Readout Test

Goal: Confirm encoder register 0x3C data is stable and indexed correctly.

Steps
	•	Echo or print encoder values continuously while:
	•	turning each wheel by hand forward/backward
	•	spinning each wheel with motor

Pass Criteria
	•	Only the intended index changes when a given wheel moves.
	•	Values increase/decrease with direction per your mapping.

Report to AI
	•	For each wheel:
	•	which encoder index moved
	•	whether forward increases or decreases
	•	approximate ticks per wheel rotation (rough estimate)

5.2 Encoder Noise / Dropout

Goal: Identify jitter at rest and missed counts at low speed.

Steps
	•	With wheels stationary: log encoder values for 30s.
	•	At very low speed: log for 30s.

Pass Criteria
	•	At rest: minimal drift (near constant).
	•	At low speed: monotonic change without big jumps.

Report to AI
	•	Any drift rate at rest
	•	Any sudden jumps or pauses

⸻

6. Power & Battery Monitoring

6.1 Voltage Register Verification

Goal: Validate 0x00 battery reading is plausible and stable.

Steps
	•	Read voltage at idle, under motor load, and after several minutes.

Pass Criteria
	•	Values align with expected battery voltage (~12–16V).
	•	Voltage dips under load are reasonable and recover.

Report to AI
	•	Voltage readings + test condition (idle vs load).
	•	Any sudden drops or nonsense values.

⸻

7. IMU Driver Verification (Bench)

7.1 Basic Topic Health

Goal: Confirm IMU publishes at expected rate with correct fields.

Steps

ros2 topic info /imu/data
ros2 topic echo /imu/data --once
ros2 topic hz /imu/data

Pass Criteria
	•	sensor_msgs/Imu published.
	•	Orientation quaternion present.
	•	Covariances populated (not all zeros unless intentional).
	•	Rate stable (choose target, e.g., 50–200 Hz).

Report to AI
	•	One message dump + Hz output.
	•	Any missing fields or zeros.

7.2 Static Bias Calibration Behavior

Goal: Confirm startup calibration reduces drift.

Steps
	1.	Place rover still.
	2.	Start IMU node; do not move for calibration duration.
	3.	Watch yaw/pitch/roll stability over 2 minutes.

Pass Criteria
	•	Pitch/roll settle and remain stable.
	•	Yaw drift exists (expected without mag), but rate should be bounded.

Report to AI
	•	Drift rate estimate (deg/min).
	•	Any jumps during calibration.

7.3 Frame Convention Test (ENU correctness)

Goal: Confirm axes match ROS ENU conventions.

Steps
	•	With rover facing “north” (choose any reference):
	•	Tilt forward/back/left/right slowly.
	•	Observe sign changes in accel and orientation.

Pass Criteria
	•	Tilts correspond to expected axis changes consistently.

Report to AI
	•	Which axes appear inverted or swapped.

⸻

8. GPS / RTK / NTRIP Verification

8.1 Device Claim / No LIBUSB_ERROR_BUSY

Goal: Confirm udev rule works and ublox_dgnss can claim device.

Steps
	•	Reboot rover.
	•	Start GPS launch:

./launch_gps.sh


	•	Check logs for USB claim issues.

Pass Criteria
	•	No LIBUSB_ERROR_BUSY.
	•	Driver starts cleanly.

Report to AI
	•	Full GPS node startup log if error.

8.2 GPS Fix Topic Health

Goal: Confirm /ublox/fix publishes and contains plausible lat/lon.

Steps

ros2 topic echo /ublox/fix --once
ros2 topic hz /ublox/fix

Pass Criteria
	•	Valid coordinates, continuous publishing.
	•	Fix status changes appropriately outdoors.

Report to AI
	•	One message dump + Hz output.
	•	Fix status and covariance values.

8.3 RTCM Flow & Subscriber Wiring

Goal: Confirm /rtcm is actually subscribed and consumed.

Steps

ros2 topic info /rtcm
ros2 topic hz /rtcm

Pass Criteria
	•	/rtcm has a publisher (NTRIP) and a subscriber (ublox node).
	•	Hz > 0 when connected.

Report to AI
	•	topic info output showing pub/sub counts.

8.4 NTRIP GGA Bridge Health (fix_to_nmea)

Goal: Confirm VRS caster receives GGA and returns corrections.

Steps

ros2 topic info /nmea
ros2 topic echo /nmea --once

Pass Criteria
	•	/nmea publishes valid $GPGGA,... sentences.
	•	Sentence length < 82 chars (spot check).
	•	NTRIP logs show periodic GGA send and RTCM receive.

Report to AI
	•	One sample GGA sentence.
	•	NTRIP log snippet showing receive bytes.

8.5 RTK Convergence Test (Outdoor)

Goal: Achieve RTK Float then RTK Fixed (if environment allows).

Steps
	•	Place rover outside with sky view.
	•	Wait 5–15 minutes while monitoring RTK status.

Pass Criteria
	•	Status transitions: Single → Float → Fixed (ideal).
	•	If not fixed, Float stable is still progress.

Report to AI
	•	Time to Float and Fixed
	•	Environment notes (trees/buildings)
	•	Any oscillation between modes

⸻

9. TF Tree Validation (Foxglove / RViz2)

9.1 TF Presence

Goal: Confirm map → odom → base_link → imu_link/gps_link exists.

Steps
	•	In Foxglove (or RViz2), visualize TF.
	•	Also run:

ros2 run tf2_tools view_frames

(then open generated PDF)

Pass Criteria
	•	All required frames present.
	•	No TF loops.
	•	Timestamps update for dynamic frames.

Report to AI
	•	TF tree screenshot + any frame missing.
	•	Any warnings about extrapolation.

9.2 Static Transform Accuracy (Quick)

Goal: Confirm sensor frame offsets are approximately correct.

Steps
	•	Measure approximate sensor positions relative to base_link.
	•	Verify in URDF/static TF values.

Pass Criteria
	•	Offsets are directionally correct and within reasonable tolerance.

Report to AI
	•	Measured offsets and current config values.

⸻

10. Wheel Odometry (Required for EKF Inputs)

If wheel odom is not implemented yet, treat this as a required development deliverable.

10.1 Odometry Topic Published

Goal: Confirm /odom exists with correct frame IDs.

Steps

ros2 topic echo /odom --once
ros2 topic hz /odom

Pass Criteria
	•	nav_msgs/Odometry published.
	•	header.frame_id = odom
	•	child_frame_id = base_link

Report to AI
	•	Message dump and frame IDs.

10.2 Straight-Line Consistency (Bench / Ground)

Goal: Confirm forward motion yields forward odom.

Steps
	•	On ground, very slow manual teleop forward 1–2 meters.
	•	Compare odom delta.

Pass Criteria
	•	X increases, Y near zero, yaw near zero.

Report to AI
	•	Odom start/end values; physical distance estimate.

10.3 Rotation Consistency

Goal: In-place rotate should produce yaw change and minimal translation.

Steps
	•	Command slow rotation 360°.

Pass Criteria
	•	Yaw changes ~360°, translation minimal.

Report to AI
	•	Observed yaw delta; drift direction.

⸻

11. EKF Localization (robot_localization)

11.1 EKF Node Health

Goal: Confirm EKF runs and outputs filtered state.

Steps

ros2 node list | grep ekf
ros2 topic echo /odometry/filtered --once
ros2 topic hz /odometry/filtered

Pass Criteria
	•	/odometry/filtered publishes steadily.
	•	Covariances non-zero and sane.

Report to AI
	•	One filtered message + any warnings.

11.2 EKF Observability Sanity

Goal: Ensure EKF isn’t “fighting itself” (wrong signs/frames).

Steps
	•	Slowly drive forward, stop, rotate.
	•	Plot:
	•	raw IMU yaw rate vs filtered yaw rate
	•	raw odom vs filtered velocity

Pass Criteria
	•	Filtered output follows inputs smoothly, no explosive divergence.

Report to AI
	•	Plot screenshot + description of divergence.

⸻

12. navsat_transform_node (Map Anchoring)

12.1 Node Running & Producing Transform

Goal: Confirm map is anchored from GPS.

Steps

ros2 node list | grep navsat
ros2 topic echo /gps/filtered --once   # (topic may vary)

Pass Criteria
	•	navsat node running.
	•	Transform map↔odom is produced (or equivalent output).

Report to AI
	•	Node list + TF screenshot.

⸻

13. Teleop Control (Ground Tests)

Only after watchdog, e-stop, and motor gating are verified.

13.1 Low-Speed Teleop Responsiveness

Goal: Confirm predictable control at low speed.

Steps
	•	Drive forward/back/strafe/rotate at low limit.
	•	Observe:
	•	actual motion
	•	encoder response
	•	/cmd_vel and wheel commands

Pass Criteria
	•	Motion matches command direction.
	•	No unexpected diagonal drift.

Report to AI
	•	Which command causes which actual motion.

13.2 Stop Behavior

Goal: Confirm rover stops quickly and reliably.

Steps
	•	Drive at low speed then command stop.
	•	Also simulate command loss (kill teleop node).

Pass Criteria
	•	Rover stops within expected time; watchdog triggers on command loss.

Report to AI
	•	Stop distance and timing estimates.

⸻

14. Foxglove Engineering UI Tests (Phase 2.5 Acceptance)

14.1 Bridge Connectivity

Goal: Foxglove connects and streams topics reliably.

Steps
	•	Connect from laptop to rover bridge.
	•	Confirm topic list and live updates.

Pass Criteria
	•	No constant disconnects.
	•	Stable streaming.

Report to AI
	•	Bridge logs + network notes.

14.2 “Field Test” Dashboard Completeness

Goal: Dashboard panels show expected signals.

Pass Criteria
	•	3D TF view correct
	•	Plots update
	•	Raw messages visible
	•	Map view works once localization is running

Report to AI
	•	Screenshot of full dashboard.

⸻

15. Autonomy Readiness Gate (Before Nav2)

15.1 Minimum Autonomy Preconditions

Must be true before Phase 3 work is enabled
	•	Motors safe: watchdog + e-stop + motor gating
	•	/tf tree correct and stable
	•	/odom and /odometry/filtered stable
	•	GPS publishing reliably outdoors
	•	Ability to record and replay rosbag2 logs

Report to AI
	•	A single “Readiness Report” summarizing pass/fail.

⸻

16. Nav2 Smoke Tests (When Phase 3 Begins)

16.1 Nav2 Bringup

Goal: Launch Nav2 without errors and verify lifecycle nodes activate.

Steps
	•	Launch Nav2.
	•	Check lifecycle transitions.

Pass Criteria
	•	All Nav2 nodes active.
	•	No costmap frame errors.

Report to AI
	•	Launch logs + node lifecycle status output.

16.2 Simple Goal Test (Outdoor)

Goal: Command a goal 5–10m away and observe behavior.

Pass Criteria
	•	Robot rotates to heading, drives smoothly, stops near goal.
	•	No oscillation or repeated recoveries.

Report to AI
	•	Video note + Foxglove plots (cmd_vel, odom, pose).

⸻

17. Teach & Repeat Tests (When Mission Controller Exists)

17.1 Recording Test

Goal: Record path only when RTK quality meets threshold.

Pass Criteria
	•	Waypoints are captured only in allowed GPS mode (e.g., RTK Fixed).
	•	Waypoints stored with timestamps and metadata.

Report to AI
	•	Recorded waypoint count + GPS status during capture.

17.2 Repeatability Test

Goal: Execute route and measure path error.

Pass Criteria
	•	Repeat path stays within acceptable corridor (define target, e.g., ≤ 0.5–1.0m initially).
	•	Behavior safe on GPS degradation (pause/stop).

Report to AI
	•	Path deviation estimate + any failure modes.

⸻

18. Bug Report Template (Copy/Paste to AI)

When something fails, provide:

Context
	•	Test name + section number:
	•	Environment: bench/indoor/outdoor
	•	Battery voltage:
	•	Software version/commit (if known):

Observed
	•	What happened:
	•	What you expected:
	•	Repro steps:
	•	Logs (paste):
	•	Foxglove screenshots (describe panel + values):

Hypothesis (optional)
	•	What you think might be wrong:

⸻

19. Final Acceptance Definition (Rover1 v1)

Rover1 is “v1 Complete” when:
	•	Manual teleop is safe and reliable
	•	Sensors publish correctly and consistently
	•	EKF produces stable filtered odometry
	•	GPS + NTRIP achieve Float/Fixed outdoors when conditions allow
	•	Foxglove dashboard provides full observability
	•	Teach & Repeat can record and replay a route safely with defined degradation behavior

⸻
