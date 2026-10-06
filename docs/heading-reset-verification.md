# Heading reset and vision heading seed: on-robot verification checklist

Covers two changes to how the field heading (the pose estimator's rotation) is set:

1. **Reset-heading (driver X) no longer touches the pose heading.** `Drivetrain.zeroGyro()` only
   re-zeros "forward" for field-relative driving. It folds the current yaw into the odometry offset,
   so `getYawForOdometry()` stays continuous and auto-align keeps aiming off the right heading.
2. **Teleop without an auto takes its heading from the cameras.** `Robot.teleopInit()` sets the
   alliance default heading (blue 180 deg, red 0 deg) as a placeholder, then calls
   `PoseEstimation.seedHeadingFromVision()`. The robot can start anywhere, facing any direction.

Neither change has been run on the robot. Simulation cannot test the vision heading seed: the sim
camera's heading always matches the gyro, so the correction there is always about 0 deg.

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
- [ ] If the seed never lands even with good tags in view, the frames disagree by more than 3 deg.
  Check the camera transforms, then consider raising `kHeadingSeedMaxSpreadDegrees` slightly.

## 5. Auto still owns heading

- [ ] Run a full auto, then go to teleop. `Heading Seeded` stays true (no seed is requested) and the
  heading carries over from the auto without a jump.

## Record for the team

- [ ] Save the `.wpilog` from the session (RIO `/home/lvuser/logs`).
- [ ] Note any changes to the `kHeadingSeed*` constants and the typical
  `HeadingSeedCorrectionDeg` values seen.
