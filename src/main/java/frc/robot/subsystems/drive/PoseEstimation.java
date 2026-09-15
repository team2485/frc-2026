package frc.robot.subsystems.drive;

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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
 * <li>The gyro owns heading: the vision heading std dev is huge, so only X/Y are corrected.</li>
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
    private final int[] acceptedCount = new int[cameras.size()];
    private final int[] rejectedCount = new int[cameras.size()];

    /** FPGA time of the last pose reset; frames captured before it are ignored. */
    private double visionResetTimestamp = 0.0;
    private double lastAcceptedTime = Double.NEGATIVE_INFINITY;
    private Optional<VisionEstimate> latestAccepted = Optional.empty();
    private Optional<Pose2d> visionOnlyPoseProjected = Optional.empty();

    private double angleToTags = 0;

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
                RejectReason reason = VisionMeasurementMath.evaluate(est, now, visionResetTimestamp,
                        robotSpeeds.omegaRadiansPerSecond);
                double age = now - est.timestampSeconds();
                Matrix<N3, N1> stdDevs = VisionMeasurementMath.stdDevs(est, fieldSpeeds, age);

                if (reason == RejectReason.ACCEPTED) {
                    poseEstimator.addVisionMeasurement(est.pose(), est.timestampSeconds(), stdDevs);
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
        }

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

        if (simGroundTruth != null) {
            simGroundTruth.resetPosition(rotation.get(), modulePosition.get(), newPose);
            simTruePose.set(newPose);
            synchronized (visionSimLock) {
                visionSim.resetRobotPose(newPose);
            }
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
