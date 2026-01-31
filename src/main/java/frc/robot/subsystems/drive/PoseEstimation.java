package frc.robot.subsystems.drive;

// import static frc.robot.Constants.PivotConstants.kPivotToRobot;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WarlordsLib.WL_CommandXboxController;
import frc.robot.Constants;

import frc.robot.Constants.VisionConstants;
// import frc.robot.commands.Interpolation.InterpolatingTable;
// import frc.robot.commands.Interpolation.ShotCalculator;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.vision.Vision;

public class PoseEstimation extends SubsystemBase {
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

    private final Supplier<Rotation2d> rotation;
    private final Supplier<SwerveModulePosition[]> modulePosition;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDrivePoseEstimator noVisionPoseEstimator;
    private final Field2d field2d = new Field2d();
    private final Vision photonEstimator = new Vision("TestCam"); // TODO
    private final Notifier photonNotifier = new Notifier(photonEstimator);
    // private final WL_CommandXboxController m_driver;
    // private final WL_CommandXboxController m_operator;

    private OriginPosition originPosition = OriginPosition.kRedAllianceWallRightSide;
    private boolean sawTag = false;
    private double angleToTags = 0;
    Supplier<ChassisSpeeds> speeds;

    // ShotCalculator yawCalculator, pitchCalculator;

    Drivetrain m_drivetrain;
    SwerveDriveKinematics m_Kinematics;
    GenericEntry visionTest;
    GenericEntry xSped;
    GenericEntry xLog;

    // boolean isOnRed;

    public PoseEstimation(Supplier<Rotation2d> rotation, Supplier<SwerveModulePosition[]> modulePosition,
            Supplier<ChassisSpeeds> chassisSpeeds,  Drivetrain m_drivetrain) {
        visionTest = Shuffleboard.getTab("Swerve").add("YSped", 10).getEntry();
        xSped = Shuffleboard.getTab("Swerve").add("XSped", 10).getEntry();
        xLog = Shuffleboard.getTab("Swerve").add("YDist", 0).getEntry();
        this.rotation = rotation;
        this.modulePosition = modulePosition;
        this.speeds = chassisSpeeds;
        // this.m_driver = m_driver;
        // this.m_operator = m_operator;

        poseEstimator = new SwerveDrivePoseEstimator(
                m_drivetrain.getKinematics(),
                rotation.get(),
                modulePosition.get(),
                new Pose2d(), stateStdDevs, visionMeasurementStdDevs);

        noVisionPoseEstimator = new SwerveDrivePoseEstimator(
                m_drivetrain.getKinematics(),
                rotation.get(),
                modulePosition.get(),
                new Pose2d());

        this.m_drivetrain = m_drivetrain;

        photonNotifier.setName("PhotonRunnable");
        photonNotifier.startPeriodic(0.02);

        // yawCalculator = new ShotCalculator();
        // pitchCalculator = new ShotCalculator();

        // isOnRed = getFieldConstants().isOnRed();
    }

    public void addDashboardWidgets(ShuffleboardTab tab) {
        // tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
        // tab.addString("Pose", this::getFormattedPose).withPosition(6, 2).withSize(2, 1);
    }

    @Override
    public void periodic() {
        poseEstimator.update(rotation.get(), modulePosition.get());
        noVisionPoseEstimator.update(rotation.get(), modulePosition.get());
        var visionPose = photonEstimator.grabLatestEstimatedPose();
        if (visionPose != null) {
            var pose2d = visionPose.estimatedPose.toPose2d();
            m_drivetrain.addVisionMeasurement(pose2d, visionPose.timestampSeconds);
            // poseEstimator.addVisionMeasurement(pose2d, visionPose.timestampSeconds);
        }

        var dashboardPose = poseEstimator.getEstimatedPosition();
        // if (originPosition == OriginPosition.kRedAllianceWallRightSide) {
        // dashboardPose = flipAlliance(dashboardPose);
        // }

        // field2d.setRobotPose(dashboardPose);
        angleToTags = getCurrentPose().getRotation().getDegrees();
        visionTest.setDouble(speeds.get().vyMetersPerSecond);
        // xSped.setDouble((isOnRed ? -1 : 1));

        // if (getNoteDetected()) m_operator.setRumble(RumbleType.kLeftRumble, 1);
        // else m_operator.setRumble(RumbleType.kLeftRumble, 0);

        // if (getNoteDetected()) m_driver.setRumble(RumbleType.kLeftRumble, 1);
        // else m_driver.setRumble(RumbleType.kLeftRumble, 0);
    }

    // private String getFormattedPose() {
    //     var pose = getCurrentPose();
    //     return String.format("(%.3f, %.3f) %.2f radians", pose.getX(), pose.getY(), pose.getRotation().getRadians());
    // }

    public Pose2d getCurrentVisionlessPose() {
        var pos = noVisionPoseEstimator.getEstimatedPosition();
        return pos;
    }

    public Pose2d getCurrentPose() {
        var pos = poseEstimator.getEstimatedPosition();

        if (pos.getX() < 0)
            pos = new Pose2d(new Translation2d(0, poseEstimator.getEstimatedPosition().getY()),
                    poseEstimator.getEstimatedPosition().getRotation());
        if (pos.getX() > VisionConstants.kFieldLengthMeters)
            pos = new Pose2d(
                    new Translation2d(VisionConstants.kFieldLengthMeters, poseEstimator.getEstimatedPosition().getY()),
                    poseEstimator.getEstimatedPosition().getRotation());

        return pos;
    }

    public Pose2d getCurrentPoseNoVision() {
        return noVisionPoseEstimator.getEstimatedPosition();
    }

    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(rotation.get(), modulePosition.get(), newPose);
    }

    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    // public double getDistanceToSpeaker() {
    //     return dist(getFieldConstants().getSpeakerPos(), getCurrentPose());
    // }

    public double dist(Pose2d pos1, Pose2d pos2) {
        double xDiff = pos2.getX() - pos1.getX();
        double yDiff = pos2.getY() - pos1.getY();
        return Math.sqrt(xDiff * xDiff + yDiff * yDiff);
    }

    // public double getAngleToSpeaker() {
    //     double deltaY = getFieldConstants().getSpeakerPos().getY() - getCurrentPose().getY();
    //     double deltaX = getFieldConstants().getSpeakerPos().getX() - getCurrentPose().getX();
    //     Pose2d anglePos = new Pose2d(deltaX, deltaY, new Rotation2d());
    //     // return Rotation2d.fromRadians(Math.atan2(deltaY, deltaX)).getDegrees();
    //     return anglePos.getTranslation().getAngle().getDegrees();
    // }

    public double getAngleFromTags() {
        if (angleToTags < -90)
            return 180 + angleToTags;
        if (angleToTags > 90)
            return -180 + angleToTags;
        return angleToTags;
    }

    // public double getAngleToAmp() {
    //     return getAngleToPos(getFieldConstants().getAmpPos());
    // }

    // public double getAngleToStage() {
    //     // if (photonEstimator.grabLatestEstimatedPose() != null)
    //     // int tagID =
    //     // photonEstimator.grabLatestEstimatedPose().targetsUsed.get(0).getFiducialId();
    //     // Rotation3d angle =
    //     // Constants.VisionConstants.kBlueTagList.get(tagID-1).pose.getRotation();
    //     // return angle.getAngle();
    //     return 0;
    // }

    public double getAngleToPos(Pose2d pos) {
        double deltaY = pos.getY() - getCurrentPose().getY();
        double deltaX = pos.getX() - getCurrentPose().getX();
        return Rotation2d.fromRadians(Math.atan2(deltaY, deltaX)).getDegrees();
    }

    public Pose2d getHubPose() {
        // return new Pose2d(VisionConstants.kBlueTagList.get(14).pose.getTranslation().toTranslation2d().plus(new Translation2d(20 * kInchesToMeters,0)), Rotation2d.kZero);\
        return new Pose2d(4.035, 4.626, new Rotation2d(0));
        // 4.626 from baseline
        // 4.035 from sideline
    }

}
