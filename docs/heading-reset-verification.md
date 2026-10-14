# Heading reset and vision heading seed: on-robot verification checklist

Covers two changes to how the field heading (the pose estimator's rotation) is set:

1. **Reset-heading (driver X) no longer touches the pose heading.** `Drivetrain.zeroGyro()` only
   re-zeros "forward" for field-relative driving. It folds the current yaw into the odometry offset,
   so `getYawForOdometry()` stays continuous and auto-align keeps aiming off the right heading.
2. **Teleop takes its heading from the cameras whenever nothing else has set it.** If
   `PoseEstimation.isHeadingKnown()` is false, `Robot.teleopInit()` sets the alliance default
   heading (blue 180 deg, red 0 deg) as a placeholder, then calls
   `PoseEstimation.seedHeadingFromVision()`. The robot can start anywhere, facing any direction.
   The heading counts as known after a pose reset (a PathPlanner auto start) or a landed seed.

Neither change has been run on the robot. Simulation cannot test the vision heading seed: the sim
camera's heading always matches the gyro, so the correction there is always about 0 deg.

## Changes since the first version of this checklist

Not yet run on the robot. The build passes, and so do the unit tests.

- **Auto-align lock check now wraps at +/-180 deg** (`TargetTracking.alignToHub`). The aim error is
  `MathUtil.angleModulus(target - current)`. Before, aiming at about 180 deg (always the case on
  red) compared e.g. +179 deg to -179 deg as 358 deg apart. The robot turned to the hub, since the PID
  already wrapped, but `Target Locked` stayed false: the windexer never fed and the controllers kept
  rumbling. Blue (aiming at about 0 deg) was not affected. `Target Tracking/Aim Error` now logs this
  wrapped error. There was no degrees-vs-radians bug in the aim path; the units were already
  consistent.
- **Seeding no longer depends on whether an auto command existed.** `getAutonomousCommand()` never
  returns null, so before, entering auto mode at all this boot (FMS, practice mode, a stray click)
  skipped the seed. The drive-forward auto never sets the pose, so the heading stayed at the boot
  value of 0 deg, which is backwards on blue. `teleopInit` now checks `isHeadingKnown()` instead.
  This also means re-enabling teleop after a seed landed keeps the seeded heading. Before, every
  teleop enable without an auto reset the heading to the placeholder and seeded again.
- **Aim PID is reset at the start of every aim** (`StateDriveToAimTransition`), so leftover integral
  from the last aim doesn't carry over.
- **Operator aim-point nudge (operator left stick) has a deadband**, `OIConstants.kOperatorAimBiasDeadband`
  (0.1). Before, stick drift moved the hub aim point, by up to 1 m at full deflection.
- **Field size updated to 2026**: `kFieldLengthMeters` 17.55 -> 16.541 and `kFieldWidthMeters`
  8.05 -> 8.069 (from WPILib's `k2026RebuiltWelded` layout). These control the vision
  outside-the-field rejection and the X clamp on the fused pose.
- **New log `Vision/HeadingSeedSpreadDeg`**: how far the last 5 seed frames disagree. If the seed
  never lands, compare this to `kHeadingSeedMaxSpreadDegrees` (3 deg).

## How the vision heading seed works

- For each accepted camera frame, it computes the camera's heading minus the estimator's heading at
  that frame's capture time (`poseEstimator.sampleAt`). Comparing at the same moment cancels camera
  latency and any turning since the frame was taken.
- A frame only counts if it sees 2 or more tags, or 1 tag closer than
  `kHeadingSeedMaxSingleTagDistanceMeters` (3 m).
- Once `kHeadingSeedFrames` (5) frames in a row agree within `kHeadingSeedMaxSpreadDegrees` (3 deg),
  their average difference is applied to the heading, once. After that the gyro owns heading again.
- Requiring 5 agreeing frames rejects a single-tag solve that comes out flipped or mirrored.
- Any pose reset (`setCurrentPose`, e.g. an auto starting) cancels a pending seed. After an auto,
  the auto's heading is kept and no seed is requested.

## What to watch

In AdvantageScope (`/RealOutputs/...`) and on the dashboard:

| Key | Meaning |
|---|---|
| `Heading Seeded` (SmartDashboard) | true once the vision seed has landed, or when none was requested |
| `Vision/HeadingSeedPending` | true while waiting for the cameras to agree |
| `Vision/HeadingSeedCorrectionDeg` | how much the seed changed the heading, logged when it lands |
| `Vision/HeadingSeedSpreadDeg` | how far the last 5 seed frames disagree; the seed lands only at 3 deg or less |
| `Target Tracking/Aim Error` (NT) | wrapped aim error in radians, always in [-pi, pi] |
| `Target Tracking/Target Locked` (NT) | true when the aim error is within tolerance; the windexer feeds only then |
| `robotPoseLog` | fused pose; its rotation is the heading auto-align uses |
| `Vision/<cam>/RawPose` | each camera's solved pose, including its heading |

Tuning constants are in `Constants.VisionConstants`.

---

## 0. Before you start

- [ ] The camera transforms are verified (pitch signs), per `docs/vision-robot-verification.md`
  section 2. The seed is only as good as the camera headings. If the seeded heading looks a few
  degrees off, check the transforms first.

## 1. Reset-heading keeps the pose heading

- [ ] Run an auto (or start teleop and let the vision seed land), then rotate the robot by hand or
  drive it to some odd angle.
- [ ] Press driver X. The `robotPoseLog` heading does **not** jump. Field-relative driving now treats
  the robot's current direction as forward.
- [ ] Right after the reset, use right trigger to aim at the hub. Auto-align points at the hub.
- [ ] Press X a few more times at different angles. The pose heading stays smooth every time.

## 2. Vision heading seed in teleop (no auto)

Power-cycle or redeploy so no auto has run this boot.

- [ ] Place the robot at a random spot and angle with at least 2 tags in view (or 1 tag within 3 m).
- [ ] Enable teleop. `Heading Seeded` goes false, then true within about a second.
- [ ] `Vision/HeadingSeedCorrectionDeg` shows how far the alliance-default placeholder was off. The
  `robotPoseLog` heading now matches the robot's real heading on the field.
- [ ] Aim at the hub with right trigger. Auto-align points at the hub.
- [ ] Repeat facing a few different directions, including roughly backwards (about 180 deg off the
  placeholder), to check the wraparound.

## 3. Seed with no tags in view

- [ ] Enable teleop with the cameras covered or facing away from all tags. `Heading Seeded` stays
  false, and the heading stays at the alliance default.
- [ ] Turn or drive until tags are in view. The seed lands then.
- [ ] Tell the drivers: until `Heading Seeded` is true, auto-align aims off a guessed heading.

## 4. Seed robustness

- [ ] With only one tag in view, at a glancing angle (single-tag solves that can flip): the seed
  either waits or lands on the correct heading. It must never land on a heading that is clearly
  wrong. If it does, lower `kHeadingSeedMaxSpreadDegrees` or `kHeadingSeedMaxSingleTagDistanceMeters`,
  or raise `kHeadingSeedFrames`.
- [ ] If the seed never lands even with good tags in view, the frames disagree by more than 3 deg;
  `Vision/HeadingSeedSpreadDeg` shows by how much. Check the camera transforms, then consider
  raising `kHeadingSeedMaxSpreadDegrees` slightly.

## 5. Auto and heading

- [ ] Run a PathPlanner auto (one that resets the pose), then go to teleop. `Heading Seeded` stays
  true (no seed is requested), and the heading carries over from the auto without a jump.
- [ ] Run the drive-forward auto (it does not reset the pose), then go to teleop. A seed is
  requested: `Heading Seeded` goes false, then true once tags are in view.
- [ ] After a seed lands, disable and re-enable teleop. The heading does **not** jump back to the
  alliance default, and no new seed is requested.

## 6. Auto-align locks on both alliances

- [ ] Set the alliance to **red** in the Driver Station. With the heading seeded, hold right trigger
  to aim at the hub. `Target Tracking/Aim Error` settles near 0 (never near +/-6.28),
  `Target Locked` goes true, the rumble stops, and the windexer feeds.
- [ ] Repeat on **blue**.
- [ ] Aim twice in a row from different spots. The second aim doesn't overshoot more than the first
  (the PID integral is reset each time).
- [ ] With the operator's left stick untouched, `AimPose` sits exactly on the hub. Pushing the stick
  past the deadband nudges it.

## Record for the team

- [ ] Save the `.wpilog` from the session (RIO `/home/lvuser/logs`).
- [ ] Note any changes to the `kHeadingSeed*` constants and the typical
  `HeadingSeedCorrectionDeg` values seen.
