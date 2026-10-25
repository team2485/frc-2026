package frc.robot.subsystems.drive;

import java.util.ArrayDeque;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionEstimate;
import frc.robot.subsystems.vision.VisionMeasurementMath;
import frc.robot.subsystems.vision.VisionMeasurementMath.RejectReason;

/**
 * The robot's pose source on the custom swerve drivetrain: wheel odometry plus the Pigeon heading,
 * corrected by the two PhotonVision cameras through a {@link SwerveDrivePoseEstimator}.
 *
 * <p>Vision fusion (see {@link #periodic()}):
 * <ul>
 * <li>Every camera frame is fused exactly once at its own capture timestamp, so the estimator
 * replays the wheel motion recorded since that frame instead of assuming a constant velocity.</li>
 * <li>Each frame's X/Y trust is reduced by how far the robot travelled along that field axis while
 * the frame was in flight (speed x age), so the two cameras' different latencies are weighed
 * individually and only along the direction of travel.</li>
 * <li>The gyro owns heading: each frame is fused at the estimator's own heading for that instant,
 * so only X/Y are corrected and a camera yaw that is slightly off cannot drag the heading.</li>
 * <li>The cameras set the heading once, from boot (so it is right before enabling), and again in
 * teleop if nothing else set it (see {@link #seedHeadingFromVision()}).</li>
 * <li>Frames captured before a pose reset (auto start, PathPlanner) are ignored so they cannot drag
 * the freshly reset pose back.</li>
 * </ul>
 */
public class PoseEstimation extends SubsystemBase {
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    /** Constructor default only; every real measurement passes its own std devs. */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

    private final Supplier<Rotation2d> rotation;
    private final Supplier<SwerveModulePosition[]> modulePosition;
    /** Robot-relative chassis speeds. */
    private final Supplier<ChassisSpeeds> speeds;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final List<Vision> cameras = List.of(
            new Vision(VisionConstants.kFrontCameraName, VisionConstants.kRobotToFrontCamera),
            new Vision(VisionConstants.kSideCameraName, VisionConstants.kRobotToSideCamera));
    /** Per camera, same order as {@link #cameras}: whether its frames may correct the pose. */
    private final boolean[] fusionEnabled = {
            VisionConstants.kFrontCameraFusionEnabled, VisionConstants.kSideCameraFusionEnabled };
    private final int[] acceptedCount = new int[cameras.size()];
    private final int[] rejectedCount = new int[cameras.size()];

    /** FPGA time of the last pose reset; frames captured before it are ignored. */
    private double visionResetTimestamp = 0.0;
    private double lastAcceptedTime = Double.NEGATIVE_INFINITY;
    private Optional<VisionEstimate> latestAccepted = Optional.empty();
    private Optional<Pose2d> visionOnlyPoseProjected = Optional.empty();

    /** True while waiting for the cameras to set the heading (see {@link #seedHeadingFromVision()}). */
    private boolean headingSeedPending = false;
    /** Recent per-frame (vision heading - estimator heading at capture), newest last. */
    private final ArrayDeque<Rotation2d> headingSeedDeltas = new ArrayDeque<>();
    /**
     * True once something has declared the real field heading this boot: a pose reset (PathPlanner
     * auto start) or a landed vision seed. False at boot and while a seed is pending.
     */
    private boolean headingKnown = false;

    private double angleToTags = 0;

    /* Camera mount-offset check: park the robot at a tape-measured pose, type it in, turn it on. */
    private static final String kOffsetCheck = "Camera Offset Check/";

    Drivetrain m_drivetrain;
    SwerveDriveKinematics m_Kinematics;

    /* Simulation only (null on the real robot). */
    private VisionSystemSim visionSim;
    /** Vision-free odometry: in simulation the wheels do not slip, so this is the true robot pose. */
    private SwerveDriveOdometry simGroundTruth;
    /** Latest true pose, handed from the main loop to the camera-sim thread. */
    private final AtomicReference<Pose2d> simTruePose = new AtomicReference<>(Pose2d.kZero);
    /** Guards visionSim between the camera-sim thread and pose resets on the main loop. */
    private final Object visionSimLock = new Object();
    private Notifier visionSimNotifier;

    public PoseEstimation(Supplier<Rotation2d> rotation, Supplier<SwerveModulePosition[]> modulePosition,
            Supplier<ChassisSpeeds> chassisSpeeds, Drivetrain m_drivetrain) {
        this.rotation = rotation;
        this.modulePosition = modulePosition;
        this.speeds = chassisSpeeds;
        this.m_drivetrain = m_drivetrain;

        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                rotation.get(),
                modulePosition.get(),
                new Pose2d(), stateStdDevs, visionMeasurementStdDevs);

        // The boot heading is just 0 deg; let the cameras fix it while the robot sits disabled.
        seedHeadingFromVision();

        SmartDashboard.putBoolean(kOffsetCheck + "Active", false);
        SmartDashboard.putNumber(kOffsetCheck + "Robot X (m)", 0.0);
        SmartDashboard.putNumber(kOffsetCheck + "Robot Y (m)", 0.0);
        SmartDashboard.putNumber(kOffsetCheck + "Robot Heading (deg)", 0.0);

        if (RobotBase.isSimulation()) {
            setUpVisionSim();
        }
    }

    public void addDashboardWidgets(ShuffleboardTab tab) {
    }

    /**
     * The most recent accepted camera pose, moved forward to now along the current velocity. Empty
     * when no frame has been accepted within {@code kMaxVisionAgeSeconds}. Diagnostic only; the
     * fused pose is {@link #getCurrentPose()}.
     */
    public Optional<Pose2d> getVisionONLYPose() {
        return visionOnlyPoseProjected;
    }

    public Optional<VisionEstimate> getLatestVisionEstimate() {
        return latestAccepted;
    }

    @Override
    public void periodic() {
        poseEstimator.update(rotation.get(), modulePosition.get());

        double now = Timer.getFPGATimestamp();
        ChassisSpeeds robotSpeeds = speeds.get();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds,
                poseEstimator.getEstimatedPosition().getRotation());

        for (int i = 0; i < cameras.size(); i++) {
            Vision cam = cameras.get(i);
            String prefix = "Vision/" + cam.getName() + "/";
            List<VisionEstimate> estimates = cam.pollEstimates();
            Logger.recordOutput(prefix + "Connected", cam.isConnected());
            Logger.recordOutput(prefix + "ResultsThisLoop", estimates.size());

            for (VisionEstimate est : estimates) {
                RejectReason reason = fusionEnabled[i]
                        ? VisionMeasurementMath.evaluate(est, now, visionResetTimestamp,
                                robotSpeeds.omegaRadiansPerSecond)
                        : RejectReason.CAMERA_DISABLED;
                double age = now - est.timestampSeconds();
                Matrix<N3, N1> stdDevs = VisionMeasurementMath.stdDevs(est, fieldSpeeds, age);

                if (reason == RejectReason.ACCEPTED && headingSeedPending && offerHeadingSeed(est)) {
                    // Heading just reset from vision; later frames this loop predate it and are dropped.
                    acceptedCount[i]++;
                } else if (reason == RejectReason.ACCEPTED) {
                    // X/Y only: give the frame the estimator's heading at capture, so the heading
                    // correction is exactly zero and the gyro alone moves the heading.
                    Optional<Pose2d> atCapture = poseEstimator.sampleAt(est.timestampSeconds());
                    if (atCapture.isPresent()) {
                        poseEstimator.addVisionMeasurement(
                                new Pose2d(est.pose().getTranslation(), atCapture.get().getRotation()),
                                est.timestampSeconds(), stdDevs);
                    }
                    acceptedCount[i]++;
                    lastAcceptedTime = now;
                    if (latestAccepted.isEmpty()
                            || est.timestampSeconds() > latestAccepted.get().timestampSeconds()) {
                        latestAccepted = Optional.of(est);
                    }
                } else {
                    rejectedCount[i]++;
                }

                Logger.recordOutput(prefix + "RawPose", est.pose());
                Logger.recordOutput(prefix + "TimestampSeconds", est.timestampSeconds());
                Logger.recordOutput(prefix + "AgeSeconds", age);
                Logger.recordOutput(prefix + "PipelineLatencyMs", est.pipelineLatencySeconds() * 1000.0);
                Logger.recordOutput(prefix + "NumTags", est.numTags());
                Logger.recordOutput(prefix + "MultiTag", est.multiTag());
                Logger.recordOutput(prefix + "AvgTagDistance", est.avgTagDistanceMeters());
                Logger.recordOutput(prefix + "MaxAmbiguity", est.maxAmbiguity());
                Logger.recordOutput(prefix + "StdDevs",
                        new double[] { stdDevs.get(0, 0), stdDevs.get(1, 0), stdDevs.get(2, 0) });
                Logger.recordOutput(prefix + "RejectReason", reason.name());
            }
            Logger.recordOutput(prefix + "Accepted", acceptedCount[i]);
            Logger.recordOutput(prefix + "Rejected", rejectedCount[i]);
            cam.getLatestCameraPose().ifPresent(p -> Logger.recordOutput(prefix + "CameraPose", p));
        }
        updateOffsetCheck();

        visionOnlyPoseProjected = latestAccepted
                .filter(est -> now - est.timestampSeconds() <= VisionConstants.kMaxVisionAgeSeconds)
                .map(est -> VisionMeasurementMath.projectForward(est.pose(), robotSpeeds,
                        now - est.timestampSeconds()));

        Logger.recordOutput("Vision/HasFreshVision", visionOnlyPoseProjected.isPresent());
        Logger.recordOutput("Vision/VisionOnlyPoseRaw", latestAccepted.map(VisionEstimate::pose).orElse(Pose2d.kZero));
        Logger.recordOutput("Vision/VisionOnlyPoseProjected", visionOnlyPoseProjected.orElse(Pose2d.kZero));
        Logger.recordOutput("Vision/VisionOnlyAgeSeconds",
                latestAccepted.map(est -> now - est.timestampSeconds()).orElse(-1.0));
        Logger.recordOutput("Vision/FieldSpeeds", fieldSpeeds);
        Logger.recordOutput("Vision/HeadingSeedPending", headingSeedPending);
        SmartDashboard.putBoolean("Heading Seeded", !headingSeedPending);
        SmartDashboard.putBoolean("Camera Positioned For Auto",
                now - lastAcceptedTime <= VisionConstants.kMaxVisionAgeSeconds);

        angleToTags = getCurrentPose().getRotation().getDegrees();
    }

    public Pose2d getCurrentPose() {
        var pos = poseEstimator.getEstimatedPosition();

        // Field-edge clamp is a real-robot safety net; skip it in sim so odometry bugs are
        // visible instead of being silently pinned to the X = 0 wall.
        if (!RobotBase.isSimulation()) {
            if (pos.getX() < 0)
                pos = new Pose2d(new Translation2d(0, poseEstimator.getEstimatedPosition().getY()),
                        poseEstimator.getEstimatedPosition().getRotation());
            if (pos.getX() > VisionConstants.kFieldLengthMeters)
                pos = new Pose2d(
                        new Translation2d(VisionConstants.kFieldLengthMeters,
                                poseEstimator.getEstimatedPosition().getY()),
                        poseEstimator.getEstimatedPosition().getRotation());
        }

        return pos;
    }

    /**
     * Resets the fused pose (auto start, PathPlanner). Camera frames captured before this instant
     * are ignored afterwards, because the reset clears the estimator history and a stale frame
     * would otherwise be applied against the new pose.
     */
    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(rotation.get(), modulePosition.get(), newPose);
        visionResetTimestamp = Timer.getFPGATimestamp();
        latestAccepted = Optional.empty();
        visionOnlyPoseProjected = Optional.empty();
        headingSeedPending = false;
        headingSeedDeltas.clear();
        headingKnown = true;

        if (simGroundTruth != null) {
            simGroundTruth.resetPosition(rotation.get(), modulePosition.get(), newPose);
            simTruePose.set(newPose);
            synchronized (visionSimLock) {
                visionSim.resetRobotPose(newPose);
            }
        }
    }

    /** Re-declares the fused heading, keeping the current X/Y. */
    public void setCurrentHeading(Rotation2d heading) {
        setCurrentPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), heading));
    }

    /**
     * Lets the cameras set the heading once (teleop without an auto, robot placed anywhere). Normal
     * fusion never corrects heading, so until a seed lands the heading is whatever it was before.
     * Cancelled by any pose reset ({@link #setCurrentPose}), e.g. an auto starting.
     */
    public void seedHeadingFromVision() {
        headingSeedPending = true;
        headingSeedDeltas.clear();
        headingKnown = false;
    }

    /**
     * Whether the field heading has been declared this boot (pose reset or landed vision seed).
     * An auto that never resets the pose (e.g. the drive-forward auto) leaves this false.
     */
    public boolean isHeadingKnown() {
        return headingKnown;
    }

    /**
     * Collects one frame toward the vision heading seed. Each frame contributes the difference
     * between its heading and the estimator's heading at the frame's capture time, so camera
     * latency and any turning since then cancel out. Once {@code kHeadingSeedFrames} recent frames
     * agree within {@code kHeadingSeedMaxSpreadDegrees}, their mean difference is applied to the
     * current heading.
     *
     * @return true if the heading was reset by this frame
     */
    private boolean offerHeadingSeed(VisionEstimate est) {
        if (!est.multiTag() && est.avgTagDistanceMeters() > VisionConstants.kHeadingSeedMaxSingleTagDistanceMeters) {
            return false;
        }
        Optional<Pose2d> atCapture = poseEstimator.sampleAt(est.timestampSeconds());
        if (atCapture.isEmpty()) {
            return false;
        }
        headingSeedDeltas.addLast(est.pose().getRotation().minus(atCapture.get().getRotation()));
        while (headingSeedDeltas.size() > VisionConstants.kHeadingSeedFrames) {
            headingSeedDeltas.removeFirst();
        }
        if (headingSeedDeltas.size() < VisionConstants.kHeadingSeedFrames) {
            return false;
        }

        // Circular mean, so deltas either side of +/-180 deg average correctly.
        double sumCos = 0, sumSin = 0;
        for (Rotation2d d : headingSeedDeltas) {
            sumCos += d.getCos();
            sumSin += d.getSin();
        }
        Rotation2d mean = new Rotation2d(sumCos, sumSin);
        double spreadDeg = 0;
        for (Rotation2d d : headingSeedDeltas) {
            spreadDeg = Math.max(spreadDeg, Math.abs(d.minus(mean).getDegrees()));
        }
        // If the seed never lands, this shows how far the frames disagree (vs kHeadingSeedMaxSpreadDegrees).
        Logger.recordOutput("Vision/HeadingSeedSpreadDeg", spreadDeg);
        if (spreadDeg > VisionConstants.kHeadingSeedMaxSpreadDegrees) {
            return false; // keep sliding the window until the frames agree
        }

        poseEstimator.resetRotation(poseEstimator.getEstimatedPosition().getRotation().plus(mean));
        visionResetTimestamp = Timer.getFPGATimestamp();
        latestAccepted = Optional.empty();
        visionOnlyPoseProjected = Optional.empty();
        headingSeedPending = false;
        headingSeedDeltas.clear();
        headingKnown = true;
        Logger.recordOutput("Vision/HeadingSeedCorrectionDeg", mean.getDegrees());
        return true;
    }

    /**
     * Reports each camera's real robot-to-camera transform, measured against a robot parked at the
     * dashboard pose. Works even when kRobotTo*Camera is wrong, so the numbers shown can replace
     * the constants as-is. Best with the robot still and two or more tags in view (MultiTag true).
     */
    private void updateOffsetCheck() {
        if (!SmartDashboard.getBoolean(kOffsetCheck + "Active", false)) {
            return;
        }
        Pose2d parked = new Pose2d(
                SmartDashboard.getNumber(kOffsetCheck + "Robot X (m)", 0.0),
                SmartDashboard.getNumber(kOffsetCheck + "Robot Y (m)", 0.0),
                Rotation2d.fromDegrees(SmartDashboard.getNumber(kOffsetCheck + "Robot Heading (deg)", 0.0)));
        for (Vision cam : cameras) {
            cam.getLatestCameraPose().ifPresent(cameraPose -> {
                Transform3d t = VisionMeasurementMath.robotToCamera(parked, cameraPose);
                Logger.recordOutput("Vision/" + cam.getName() + "/MeasuredRobotToCamera", t);
                SmartDashboard.putString(kOffsetCheck + cam.getName(), String.format(
                        "x %.1f in, y %.1f in, z %.1f in | roll %.1f, pitch %.1f, yaw %.1f deg",
                        Units.metersToInches(t.getX()), Units.metersToInches(t.getY()),
                        Units.metersToInches(t.getZ()), Math.toDegrees(t.getRotation().getX()),
                        Math.toDegrees(t.getRotation().getY()), Math.toDegrees(t.getRotation().getZ())));
            });
        }
    }

    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    public double dist(Pose2d pos1, Pose2d pos2) {
        double xDiff = pos2.getX() - pos1.getX();
        double yDiff = pos2.getY() - pos1.getY();
        return Math.sqrt(xDiff * xDiff + yDiff * yDiff);
    }

    public double getAngleFromTags() {
        if (angleToTags < -90)
            return 180 + angleToTags;
        if (angleToTags > 90)
            return -180 + angleToTags;
        return angleToTags;
    }

    public double getAngleToPos(Pose2d pos) {
        double deltaY = pos.getY() - getCurrentPose().getY();
        double deltaX = pos.getX() - getCurrentPose().getX();
        return Rotation2d.fromRadians(Math.atan2(deltaY, deltaX)).getDegrees();
    }

    /* ---------------------------------------------------------------------------------------- */
    /* Simulation: synthetic PhotonVision frames so the two-camera latency handling can be       */
    /* exercised in simulateJava. The two cameras get deliberately different latencies.          */
    /* ---------------------------------------------------------------------------------------- */

    private static final double[] kSimCameraLatencyMs = { 40.0, 70.0 };

    private void setUpVisionSim() {
        visionSim = new VisionSystemSim("main");
        try {
            visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded));
        } catch (Exception e) {
            DriverStation.reportError("Vision sim: failed to load the AprilTag field layout", e.getStackTrace());
        }

        for (int i = 0; i < cameras.size(); i++) {
            SimCameraProperties props = new SimCameraProperties();
            props.setCalibration(1280, 800, Rotation2d.fromDegrees(75));
            props.setCalibError(0.25, 0.08);
            props.setFPS(30);
            props.setAvgLatencyMs(kSimCameraLatencyMs[i % kSimCameraLatencyMs.length]);
            props.setLatencyStdDevMs(8);

            PhotonCameraSim camSim = new PhotonCameraSim(cameras.get(i).getCamera(), props);
            camSim.enableRawStream(false);
            camSim.enableProcessedStream(false);
            camSim.enableDrawWireframe(false);
            // Tags the real fusion would reject anyway (see kVisionMaxDistanceMeters) are not worth
            // a simulated PnP solve.
            camSim.setMaxSightRange(VisionConstants.kVisionMaxDistanceMeters + 1.0);
            visionSim.addCamera(camSim, cameras.get(i).getRobotToCamera());
        }

        simGroundTruth = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, rotation.get(),
                modulePosition.get());

        /* PhotonLib's camera simulation runs an OpenCV PnP solve per visible tag and costs ~20 ms per
         * frame on a desktop, which overruns the 20 ms loop when run from simulationPeriodic(). It gets
         * its own thread; the frames still reach Vision through NetworkTables exactly as on the robot.
         * Only the true pose crosses threads (simTruePose); the gyro/module suppliers stay on the
         * main loop because Drivetrain's median filter is not thread-safe. */
        visionSimNotifier = new Notifier(() -> {
            synchronized (visionSimLock) {
                visionSim.update(simTruePose.get());
            }
        });
        visionSimNotifier.setName("VisionSim");
        visionSimNotifier.startPeriodic(Constants.kTimestepSeconds);
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim == null) {
            return;
        }
        simGroundTruth.update(rotation.get(), modulePosition.get());
        Pose2d truePose = simGroundTruth.getPoseMeters();
        simTruePose.set(truePose);
        Logger.recordOutput("Vision/SimGroundTruthPose", truePose);
    }
}
