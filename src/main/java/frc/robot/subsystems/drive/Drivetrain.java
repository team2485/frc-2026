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
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            absoluteGyroPosition = 180;
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

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
            Translation2d centerOfRotation) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        -rotation,
                        Rotation2d.fromDegrees(gyro.getYaw().refresh().getValueAsDouble() * -1) // TODO: all gyro values
                )
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation),
                centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        recordSimCommandedOmega(fieldRelative ? -rotation : rotation);
        absoluteGyroPos.setDouble(getChassisSpeeds().vyMetersPerSecond);
        currentGyroPos.setDouble(getYaw().times(-1).getDegrees());

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
                        -rotation,
                        absoluteRotation)
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        recordSimCommandedOmega(fieldRelative ? -rotation : rotation);
        absoluteGyroPos.setDouble(absoluteGyroPosition);
        currentGyroPos.setDouble(getYaw().times(-1).getDegrees());

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

        driveWithSuppliedRotation(new Translation2d(-speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                speeds.omegaRadiansPerSecond, false, true, Rotation2d.fromDegrees(getYawAbsolute().getDegrees() % 180));
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

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = new SwerveModulePosition(mod.getPosition().distanceMeters, mod.getCanCoder());
        }
        return positions;
    }

    public SwerveModulePosition[] getModulePositionsInverted() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = new SwerveModulePosition(-mod.getPosition().distanceMeters,
                    mod.getCanCoder());
        }
        return positions;
    }

    /**
     * Module positions fed to the pose estimator. The real robot keeps the historical distance
     * inversion of {@link #getModulePositionsInverted()}; simulation uses the true (non-inverted)
     * positions so the rendered pose tracks the commanded motion instead of driving into the
     * X = 0 field edge.
     * <p>
     * TODO: the uniform distance negation is almost certainly a hardware band-aid for another
     * sign error (gyro or drive-encoder polarity). Verify on blocks and unify the two paths.
     */
    public SwerveModulePosition[] getModulePositionsForOdometry() {
        if (RobotBase.isSimulation()) {
            /* Use the steer control angle (getAngle(), which tracks the setpoint exactly in sim)
             * instead of getCanCoder() -- the Phoenix CANcoder sim does not resolve
             * getAbsolutePosition() predictably here, and its bogus per-module angles skew the
             * estimated translation by ~21 deg. Distance is negated to match the drive-position
             * sign convention (forward -> +X). */
            SwerveModulePosition[] positions = new SwerveModulePosition[4];
            for (SwerveModule mod : mSwerveMods) {
                SwerveModulePosition p = mod.getPosition();
                positions[mod.moduleNumber] = new SwerveModulePosition(-p.distanceMeters, p.angle);
            }
            return positions;
        }
        return getModulePositionsInverted();
    }

    /**
     * Heading fed to the pose estimator. In simulation this matches the field-relative reference
     * {@code drive()} uses ({@code gyro.getYaw() * -1}) so the rendered pose and the drive command
     * share one convention and stay consistent after a rotation or a {@code zeroGyro()}. The real
     * robot keeps the historical {@link #getYawMod()} path (which pairs with the distance inversion
     * in {@link #getModulePositionsInverted()}).
     */
    public Rotation2d getYawForOdometry() {
        return RobotBase.isSimulation() ? getYaw().times(-1) : getYawMod();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void zeroGyro() {
        absoluteGyroPosition += getYaw().times(-1).getDegrees();
        gyro.setYaw(0);
    }

    public void autoGyro() {
        gyro.setYaw(180);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro)
                ? Rotation2d.fromDegrees(filter.calculate(360 - gyro.getYaw().refresh().getValueAsDouble()))
                : Rotation2d.fromDegrees(filter.calculate(gyro.getYaw().refresh().getValueAsDouble()));
    }

    public Rotation2d getYawAbsolute() {
        return Rotation2d.fromDegrees(absoluteGyroPosition).minus(getYaw()).times(-1);
    }

    public Rotation2d getYawMod() {
        return Rotation2d.fromDegrees(getYawAbsolute().getDegrees() % 360);
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
