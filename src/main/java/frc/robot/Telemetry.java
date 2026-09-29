package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.PoseEstimation;
import frc.robot.subsystems.drive.SwerveModule;

public class Telemetry {
    private final double MaxSpeed;

    private final Drivetrain m_drivetrain;
    private final PoseEstimation m_poseEstimation;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     *
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed, Drivetrain drivetrain, PoseEstimation poseEstimation) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
        m_drivetrain = drivetrain;
        m_poseEstimation = poseEstimation;
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];

    /** Pull the drive state from the drivetrain/pose estimation and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize() {
        Pose2d pose = m_poseEstimation.getCurrentPose();
        ChassisSpeeds speeds = m_drivetrain.getChassisSpeeds();
        SwerveModuleState[] moduleStates = m_drivetrain.getModuleStates();
        SwerveModulePosition[] modulePositions = m_drivetrain.getModulePositions();

        /* Telemeterize the swerve drive state */
        drivePose.set(pose);
        driveSpeeds.set(speeds);
        driveModuleStates.set(moduleStates);
        driveModulePositions.set(modulePositions);
        driveTimestamp.set(Timer.getFPGATimestamp());

        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, moduleStates);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, modulePositions);

        for(int i = 0; i < 4; i++) {
            SwerveModule module = m_drivetrain.mSwerveMods[i];
            Logger.recordOutput("Drive/" + "Module" + i + "_steerCurrent", module.getAngleMotor().getStatorCurrent().getValueAsDouble());
            Logger.recordOutput("Drive/" + "Module" + i + "_driveCurrent", module.getDriveMotor().getStatorCurrent().getValueAsDouble());
            Logger.recordOutput("Drive/" + "Module" + i + "_steerVoltage", module.getAngleMotor().getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Drive/" + "Module" + i + "_driveVoltage", module.getDriveMotor().getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Drive/" + "Module" + i + "_steerVelocity", module.getAngleMotor().getVelocity().getValueAsDouble());
            Logger.recordOutput("Drive/" + "Module" + i + "_driveVelocity", module.getDriveMotor().getVelocity().getValueAsDouble());
        }

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");

        m_poseArray[0] = pose.getX();
        m_poseArray[1] = pose.getY();
        m_poseArray[2] = pose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(moduleStates[i].angle);
            m_moduleDirections[i].setAngle(moduleStates[i].angle);
            m_moduleSpeeds[i].setLength(moduleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        }
    }
}
