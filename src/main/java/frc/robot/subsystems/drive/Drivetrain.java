package frc.robot.subsystems.drive;

import static frc.robot.Constants.Swerve.swerveKinematics;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    public Pigeon2 gyro = new Pigeon2(Constants.Swerve.pigeonID, "Drive");
    GenericEntry absoluteGyroPos;
    GenericEntry currentGyroPos;
    public RobotConfig pathplannerConfig;

    private double absoluteGyroPosition = 0;

    /* Simulation-only: omega (rad/s) from the most recent drive command, applied to the Pigeon 2
     * sim state as an incremental yaw each simulationPeriodic(). Using the commanded value (not one
     * re-derived from noisy module states) keeps a pure translation command drift-free; applying it
     * incrementally (addYaw, not setRawYaw) lets gyro.setYaw()/zeroGyro() work normally in sim.
     * Consumed and cleared each simulationPeriodic(). */
    private double simCommandedOmegaRadPerSec = 0;

    public SwerveModule[] mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    public Pigeon2Configuration config = new Pigeon2Configuration();

    private MedianFilter filter = new MedianFilter(5);

    public Drivetrain() {
        // Shuffleboard.getTab("Autos").addString("ChassisSpeedOut", ::toString);

        // gyro.configFactoryDefault();
        absoluteGyroPos = Shuffleboard.getTab("Swerve").add("AbsoluteGyroPos", 0).getEntry();
        currentGyroPos = Shuffleboard.getTab("Swerve").add("CurrentGyroPos", 0).getEntry();
        gyro.reset();
        absoluteGyroPosition = allianceHeadingOffsetDegrees();
        try {
            pathplannerConfig = RobotConfig.fromGUISettings();
            // System.out.println("asdfadsfasdf" + pathplannerConfig.toString());
        } catch (Exception e) {
            // Handle exception as needed

            e.printStackTrace();
        }
        // mSwerveMods = new SwerveModule[] {
        // new SwerveModule(0, Constants.Swerve.Mod0.constants),
        // new SwerveModule(1, Constants.Swerve.Mod1.constants),
        // new SwerveModule(2, Constants.Swerve.Mod2.constants),
        // new SwerveModule(3, Constants.Swerve.Mod3.constants)
        // };

        // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics,
        // getYaw().times(-1), getModulePositions());
    }

    /**
     * Drives in the standard WPILib frame: +X forward, +Y left, rotation CCW+ (rad/s). Field
     * relative uses the driver-zeroed gyro heading (see {@link #zeroGyro()}), which the Pigeon 2
     * already reports CCW+.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
            Translation2d centerOfRotation) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getRawYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation),
                centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        recordSimCommandedOmega(rotation);
        absoluteGyroPos.setDouble(absoluteGyroPosition);
        currentGyroPos.setDouble(getRawYaw().getDegrees());

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {

        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] targetStates = swerveKinematics.toSwerveModuleStates(targetSpeeds);

        // SwerveDriveKinematics.desaturateWheelSpeeds(targetStates,
        // Constants.Swerve.maxSpeed);

        recordSimCommandedOmega(targetSpeeds.omegaRadiansPerSecond);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(targetStates[mod.moduleNumber], false);
        }

    }

    /**
     * Simulation only: stashes the commanded chassis angular rate (rad/s) so
     * {@link #simulationPeriodic()} can integrate the Pigeon 2 heading straight from the command.
     * Taking it from the source rate (not re-deriving it from optimized module states) keeps a
     * pure-translation command at exactly zero and avoids the first-tick transient where the
     * steer feedback is still the pre-drive {@code resetToAbsolute} seed. No-op on a real robot.
     */
    private void recordSimCommandedOmega(double omegaRadPerSec) {
        if (RobotBase.isSimulation()) {
            simCommandedOmegaRadPerSec = omegaRadPerSec;
        }
    }

    public void driveWithSuppliedRotation(Translation2d translation, double rotation, boolean fieldRelative,
            boolean isOpenLoop, Rotation2d absoluteRotation) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        absoluteRotation)
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        recordSimCommandedOmega(rotation);
        absoluteGyroPos.setDouble(absoluteGyroPosition);
        currentGyroPos.setDouble(getRawYaw().getDegrees());

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    // public void driveAuto(ChassisSpeeds speeds) {
    // double rot =
    // -rotationOverrideController.calculate(getYawAbsolute().getDegrees() % 180,
    // rotationOverride.getDegrees());
    // if (speeds.vxMetersPerSecond < .5 && speeds.vyMetersPerSecond < .5)
    // rot = 0;
    // speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getYawAbsolute());
    // driveWithSuppliedRotation(new Translation2d(speeds.vxMetersPerSecond,
    // -speeds.vyMetersPerSecond),rot, true, false,
    // Rotation2d.fromDegrees(getYawAbsolute().getDegrees() % 180));
    // }

    public void driveAuto(ChassisSpeeds speeds) {
        // if(Math.abs(speeds.vxMetersPerSecond) > 0.5) speeds.vxMetersPerSecond = 0.5 *
        // Math.signum(speeds.vxMetersPerSecond);/// NOTE: NEEDS SIGN CHANGER
        // if(Math.abs(speeds.vyMetersPerSecond) > 0.5) speeds.vyMetersPerSecond = 0.5 *
        // Math.signum(speeds.vyMetersPerSecond); // HARD SPEED LIMIT!!

        /* PathPlanner hands us robot-relative speeds in the standard frame; pass them straight
         * through (open loop, matching teleop, since the drive velocity loop has no kV tuned). */
        drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                speeds.omegaRadiansPerSecond, false, true, new Translation2d());
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        recordSimCommandedOmega(
                Constants.Swerve.swerveKinematics.toChassisSpeeds(desiredStates).omegaRadiansPerSecond);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public double getPitch() {
        // return gyro.getRoll() + 4;
        return gyro.getPitch().refresh().getValueAsDouble();
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * Module positions for odometry: drive distance (forward positive) and the steer motor's
     * integrated angle, which is seeded from the CANcoder by {@link #resetToAbsolute()}. The
     * integrated angle is used rather than the CANcoder directly so the same value drives both
     * control and odometry (and because the Phoenix CANcoder sim is unreliable in simulation).
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Heading fed to the pose estimator: the field heading, i.e. the gyro plus the alliance
     * offset applied by {@link #zeroGyro()}. Same frame as {@code drive()} in both sim and real.
     */
    public Rotation2d getYawForOdometry() {
        return getYawAbsolute();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Field heading the robot is at when the driver presses reset-heading, i.e. squared up facing
     * away from their own driver station. Kept identical to the old Phoenix operator-perspective
     * constants this robot's autos/aiming were built against (Blue 180 deg, Red 0 deg).
     */
    private static double allianceHeadingOffsetDegrees() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Blue
                ? 180
                : 0;
    }

    /**
     * Reset-heading: makes the robot's current direction "forward" for field-relative driving and
     * re-declares the odometry heading as the alliance forward heading (same as Phoenix
     * seedFieldCentric with the operator perspective set). Only the gyro is touched; the modules
     * are not moved.
     */
    public void zeroGyro() {
        absoluteGyroPosition = allianceHeadingOffsetDegrees();
        gyro.setYaw(0);
        filter.reset();
    }

    public void autoGyro() {
        gyro.setYaw(180);
    }

    /** Driver-zeroed gyro yaw, CCW+, unfiltered (used for field-relative drive commands). */
    public Rotation2d getRawYaw() {
        double yawDeg = gyro.getYaw().refresh().getValueAsDouble();
        return Rotation2d.fromDegrees(Constants.Swerve.invertGyro ? -yawDeg : yawDeg);
    }

    /** Driver-zeroed gyro yaw, CCW+, median filtered. */
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(filter.calculate(getRawYaw().getDegrees()));
    }

    /** Field heading: driver-zeroed yaw plus the alliance offset set by {@link #zeroGyro()}. */
    public Rotation2d getYawAbsolute() {
        return Rotation2d.fromDegrees(absoluteGyroPosition).plus(getYaw());
    }

    public void setCustomYawAbsolute(double yawAbsolute) {
        absoluteGyroPosition = yawAbsolute;
    }

    public void setCustomYaw(double yaw) {
        gyro.setYaw(yaw);
    }

    // public double continiousLoop(double value, double min, double max) {
    // double mapValue = value / 360;
    // // value of 360 is 1 time
    // if (value < min)
    // return max - (min - value);
    // return value;
    // }

    public void resetToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        double dtSeconds = Constants.kTimestepSeconds;

        for (SwerveModule mod : mSwerveMods) {
            mod.updateSimState(dtSeconds);
        }

        /* Apply the most recent commanded rotation rate to the Pigeon 2 sim state as an
         * incremental yaw so getYaw() and field-relative driving behave in simulation. Using
         * addYaw() (not setRawYaw with an accumulator) leaves gyro.setYaw()/zeroGyro() free to
         * set the absolute heading. Cleared each tick so heading holds when no command runs. */
        double yawDeltaDeg = Math.toDegrees(simCommandedOmegaRadPerSec) * dtSeconds;
        simCommandedOmegaRadPerSec = 0;

        var gyroSim = gyro.getSimState();
        gyroSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        gyroSim.addYaw(yawDeltaDeg);
    }

    // for(SwerveModule mod : mSwerveMods){
    // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder",
    // mod.getCanCoder().getDegrees());
    // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
    // mod.getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
    // mod.getState().speedMetersPerSecond);

    // }
}
