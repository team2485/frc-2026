# Vision fusion: on-robot verification checklist

Covers the two-camera PhotonVision fusion in `PoseEstimation`. SideCam fusion is currently off; see section 2.
Everything below was verified in simulation only; these are the checks that need the physical robot.

All logged values live under `/RealOutputs/Vision/...` in AdvantageScope (connect over NetworkTables,
or open the `.wpilog` from the RIO afterwards). The fused pose is `/RealOutputs/robotPoseLog`.

| Key (per camera: `FrontCam`, `SideCam`) | Meaning |
|---|---|
| `Vision/<cam>/Connected` | PhotonLib heartbeat from that camera |
| `Vision/<cam>/Accepted`, `Rejected` | running counts of frames fused / dropped |
| `Vision/<cam>/RejectReason` | `ACCEPTED`, `BEFORE_RESET`, `FUTURE`, `STALE`, `TOO_FAR`, `AMBIGUOUS`, `ROTATING_TOO_FAST`, `CAMERA_DISABLED` |
| `Vision/<cam>/RawPose` | pose solved from the last frame, before fusion |
| `Vision/<cam>/AgeSeconds` | now minus capture time (what the fusion uses) |
| `Vision/<cam>/PipelineLatencyMs` | coprocessor latency reported by PhotonVision |
| `Vision/<cam>/NumTags`, `MultiTag`, `AvgTagDistance`, `MaxAmbiguity` | solve quality inputs |
| `Vision/<cam>/StdDevs` | `[x, y, theta]` trust used for that frame (bigger = trusted less) |
| `Vision/VisionOnlyPoseRaw` / `VisionOnlyPoseProjected` | newest accepted frame, raw and moved forward to now |
| `Vision/FieldSpeeds` | chassis speeds in the field frame |
| `Vision/<cam>/CameraPose` | field pose of the camera itself from the tags (independent of the transform constants) |
| `Vision/<cam>/MeasuredRobotToCamera` | real transform while `Camera Offset Check/Active` is on |

Tuning constants are in `Constants.VisionConstants`; camera transforms are `kRobotToFrontCamera` and
`kRobotToSideCamera`.

---

## 0. Before you start

- [ ] PhotonVision camera names are exactly `FrontCam` and `SideCam` (they must match `kFrontCameraName` / `kSideCameraName`).
- [ ] Both pipelines are in AprilTag mode with **multi-target estimation enabled** and the **2026 welded field layout** loaded on the coprocessor. Without multi-tag, every frame falls back to the single-tag path and `MultiTag` will always be false.
- [ ] The Driver Station shows no PhotonLib time-sync alert after ~10 s of being connected.
- [ ] Deploy, enable briefly, and confirm no "Loop time of 0.02s overrun" spam on the DS console. `PoseEstimation.periodic()` should be well under 20 ms.

## 1. Frames arrive and timestamps are sane (robot stationary, tags in view)

- [ ] `Connected` is true for both cameras.
- [ ] `Accepted` counts up at roughly the camera frame rate (about 30 per second per camera) while tags are in view, and stops when you cover the lens.
- [ ] `AgeSeconds` sits around 0.02 to 0.08 s and is never more than about 0.02 s negative.
- [ ] `RejectReason` never shows `FUTURE` or `STALE` while stationary.

If this fails:
- `Connected` false: camera name mismatch or the coprocessor is not on the robot network.
- `FUTURE` / large negative age: time sync between coprocessor and RIO is broken (check the PhotonVision time-sync status).
- `STALE`: NetworkTables lag or the coprocessor is overloaded (check `PipelineLatencyMs`).

## 2. Camera mount calibration (Camera Offset Check)

### Current state

| Camera | Transform in code (`Constants.VisionConstants`) | Fused into the pose? |
|---|---|---|
| FrontCam | x -13.5 in, y +3.2 in, z 17.7 in, pitch 15 deg, yaw 0 (original measurement) | yes (`kFrontCameraFusionEnabled = true`) |
| SideCam | x +12.5 in, y +13.5 in, z 12.6 in, pitch 18.5 deg, yaw +90 (unverified) | **no** (`kSideCameraFusionEnabled = false`) |

SideCam is still read and logged while disabled. Its frames show `RejectReason` = `CAMERA_DISABLED`
and never move the pose or the heading seed. That way it can be calibrated with the steps below
without affecting the robot. Neither camera's pitch sign has been verified (positive pitch = tilted down).

Recalibrating a camera in the PhotonVision UI only fixes the lens model. It does not tell the robot
code where the camera is mounted; that is the `kRobotTo*Camera` transform, and this check measures it.

### How the check works

The tags tell PhotonVision exactly where the camera itself is on the field, whatever the code's
transform says. The code logs that as `Vision/<cam>/CameraPose`. If you also tell the code exactly where
the robot is, it can work out the real robot-to-camera transform. So the readout is correct even while
the constants are wrong.

### Steps

1. **Pick a spot** 2 to 3 m from at least two AprilTags that the camera you are calibrating can see.
   Check `Vision/<cam>/MultiTag` is true. Single-tag solves are noisier and can come out flipped.
2. **Park the robot flat and still** at that spot, squared to the field walls if you can (heading
   exactly 0, 90, 180 or -90 deg is easiest to get right).
3. **Measure the robot centre** in field metres, using the blue-origin WPILib field frame:
   - X = distance from the blue alliance wall, toward red.
   - Y = distance from the field edge on your right when you stand behind the blue driver station.
   - Heading: 0 deg = robot front (the end that drives forward) pointing at the red wall, +90 deg =
     pointing left from the blue driver station's view.
   - Robot centre = bumper edge + half the bumper-to-bumper length. An error of 2 cm here shows up as
     2 cm in the result, so measure carefully.
4. **Enter it on SmartDashboard**: `Camera Offset Check/Robot X (m)`, `Robot Y (m)`,
   `Robot Heading (deg)`, then tick `Camera Offset Check/Active`.
5. **Read** `Camera Offset Check/FrontCam` and `Camera Offset Check/SideCam`. Each shows
   `x .. in, y .. in, z .. in | roll .., pitch .., yaw .. deg`. That is the camera position from the
   robot centre (+x forward, +y left, +z up from the floor) and its angles, in the same order as
   `new Rotation3d(roll, pitch, yaw)`. The values wobble by a few tenths; read the middle of the range.
   They are also logged as `Vision/<cam>/MeasuredRobotToCamera`.
6. **Repeat from a second spot** (different distance or angle). The two readings should agree within
   about 1 in and 2 deg. If they don't, the parked pose was mis-measured or the lens calibration is poor.
7. **Copy the averaged reading into the constant**, keeping the units:
   ```java
   new Transform3d(
           new Translation3d(Units.inchesToMeters(x), Units.inchesToMeters(y), Units.inchesToMeters(z)),
           new Rotation3d(Units.degreesToRadians(roll), Units.degreesToRadians(pitch),
                   Units.degreesToRadians(yaw)));
   ```
   Roll should come out near 0. A pitch with the opposite sign to the constant means the old pitch
   sign was wrong.
8. **Untick `Active`**, redeploy, and re-park at one of the spots. `Vision/<cam>/RawPose` should now
   match the tape-measured pose within about 5 to 10 cm.
9. **Re-enable SideCam** once its transform is measured: set `kSideCameraFusionEnabled = true`, redeploy.

### After both cameras are calibrated

- [ ] With **both** cameras seeing tags at the same time and the robot stationary, the two `RawPose` values agree within about 10 cm. Disagreement here is a transform error, not a latency problem.
- [ ] Rotate the robot 90 deg in place and re-check: the fused pose heading tracks the gyro (vision never moves the heading; each frame is fused at the estimator's own heading), and `RawPose` X/Y stays put.

## 3. Reject gates behave

- [ ] Only one tag in view, more than 5 m away: `RejectReason` shows `TOO_FAR`.
- [ ] One tag in view at a glancing angle (single-tag solve): `AMBIGUOUS` appears when `MaxAmbiguity` exceeds 0.2, and the fused pose does not jump.
- [ ] Spin the robot fast (over about 115 deg/s): `ROTATING_TOO_FAST` appears and the fused pose does not twitch.
- [ ] Two or more tags in view: `MultiTag` is true and frames are `ACCEPTED` even when `MaxAmbiguity` is high.

## 4. Moving: latency compensation and velocity weighting

Drive past tags at speed (2 to 3 m/s), first sideways then forwards, with both cameras seeing tags.

- [ ] `StdDevs[0]` and `[1]` grow only along the axis you are moving on (field frame), and return to the stationary value when you stop.
- [ ] The two cameras' `RawPose` values differ by roughly speed x the difference in their `AgeSeconds`; that is expected.
- [ ] `robotPoseLog` stays smooth: no jumps larger than a few centimetres when a camera's frames arrive.
- [ ] `VisionOnlyPoseProjected` overlays `robotPoseLog` while moving; `VisionOnlyPoseRaw` trails behind by about speed x age.
- [ ] Stop hard next to a tag: the fused pose does not overshoot or snap back.

If the fused pose lags or leads the raw camera pose while moving, the timestamps are wrong (time sync), not the tuning.

## 5. Pose resets

- [ ] Press reset-heading (driver X): only field-relative "forward" changes; the pose heading does not jump. Full checks are in `docs/heading-reset-verification.md`.
- [ ] Start an auto: after PathPlanner sets the starting pose, there is no position jump back toward the old pose in the next second. `RejectReason` may show `BEFORE_RESET` for a few frames right after, then `ACCEPTED` resumes.
- [ ] Reminder: the gyro owns heading during normal fusion. The exception is teleop without an auto, where the cameras set the heading once (see `docs/heading-reset-verification.md`).

## 6. Long drive and recovery

- [ ] Drive across the field with no tags in view, then bring tags back into view: the pose corrects within about 1 s, without a visible snap.
- [ ] If corrections look jittery or snap too hard: raise `kXYStdDevCoefficient` (0.3 default; 0.5 to 0.8 trusts vision less).
- [ ] If odometry drift is not being corrected fast enough: lower `kXYStdDevCoefficient` (0.15 to 0.2) or raise `kVisionMaxDistanceMeters`.
- [ ] If moving frames still cause wobble: raise `kMotionStdDevGain` (1.0 default).

## 7. Aiming still works

- [ ] Target-lock aiming (right trigger) locks on and the hood/flywheel distance lookups behave as before at a few distances, since all of them read `PoseEstimation.getCurrentPose()`.
- [ ] Run one full auto to confirm PathPlanner follows paths and ends where expected.

## Record for the team

- [ ] Save the `.wpilog` from the session (RIO `/home/lvuser/logs`).
- [ ] Note the final camera transforms (including pitch signs) and any constant changes in `Constants.VisionConstants`.
