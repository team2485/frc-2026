# Vision fusion: on-robot verification checklist

Covers the two-camera PhotonVision fusion in `PoseEstimation` (branch `feat/drivetrain`).
Everything below was verified in simulation only; these are the checks that need the physical robot.

All logged values live under `/RealOutputs/Vision/...` in AdvantageScope (connect over NetworkTables,
or open the `.wpilog` from the RIO afterwards). The fused pose is `/RealOutputs/robotPoseLog`.

| Key (per camera: `FrontCam`, `SideCam`) | Meaning |
|---|---|
| `Vision/<cam>/Connected` | PhotonLib heartbeat from that camera |
| `Vision/<cam>/Accepted`, `Rejected` | running counts of frames fused / dropped |
| `Vision/<cam>/RejectReason` | `ACCEPTED`, `BEFORE_RESET`, `FUTURE`, `STALE`, `TOO_FAR`, `AMBIGUOUS`, `ROTATING_TOO_FAST` |
| `Vision/<cam>/RawPose` | pose solved from the last frame, before fusion |
| `Vision/<cam>/AgeSeconds` | now minus capture time (what the fusion uses) |
| `Vision/<cam>/PipelineLatencyMs` | coprocessor latency reported by PhotonVision |
| `Vision/<cam>/NumTags`, `MultiTag`, `AvgTagDistance`, `MaxAmbiguity` | solve quality inputs |
| `Vision/<cam>/StdDevs` | `[x, y, theta]` trust used for that frame (bigger = trusted less) |
| `Vision/VisionOnlyPoseRaw` / `VisionOnlyPoseProjected` | newest accepted frame, raw and moved forward to now |
| `Vision/FieldSpeeds` | chassis speeds in the field frame |

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

## 2. Camera transforms and the pitch sign (tape measure)

The old code used degrees divided by pi/180, so the pitch values have **never** been valid on the robot.
WPILib positive pitch tilts the camera down.

- [ ] Park the robot at a measured spot 2 to 3 m from a tag, squared to the field, with only the front camera seeing tags. Compare `Vision/FrontCam/RawPose` X and Y with the tape measure. Expect agreement within about 10 cm.
- [ ] If it is off by tens of centimetres, or the reported position slides as you tilt the robot slightly, negate the pitch in `kRobotToFrontCamera` (`Units.degreesToRadians(-15)`), redeploy, re-measure.
- [ ] Repeat for the side camera with `kRobotToSideCamera` (pitch 18.5 deg, yaw +90 deg). Also confirm the side camera's pose does not flip sides when you move the robot left/right; a mirrored result means the yaw sign is wrong.
- [ ] With **both** cameras seeing tags at the same time and the robot stationary, the two `RawPose` values agree within about 10 cm. Disagreement here is a transform error, not a latency problem.
- [ ] Rotate the robot 90 deg in place and re-check: the fused pose heading tracks the gyro (vision does not correct heading by design), and `RawPose` X/Y stays put.

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

- [ ] Press reset-heading (driver X): the pose heading changes to the alliance forward heading and there is no position jump back toward the old pose in the next second. `RejectReason` may show `BEFORE_RESET` for a few frames right after.
- [ ] Start an auto: after PathPlanner sets the starting pose, the same holds. Frames captured before the reset are dropped, then `ACCEPTED` resumes.
- [ ] Reminder: the gyro owns heading. If the driver squares up badly before reset-heading, vision will **not** fix the heading. Lower `kVisionThetaStdDevRadians` only if the team decides vision should correct heading.

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
