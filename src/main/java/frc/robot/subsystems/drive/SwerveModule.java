package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drive.CTREConfigs;
import frc.util.CTREModuleState;
import frc.util.Conversions;
import frc.util.CTREModuleState;
import frc.util.SwerveModuleConstants;

import static frc.robot.Constants.Swerve.*;

import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;


public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private VelocityVoltage mDriveVelocityVoltage = new VelocityVoltage(0);
    private PositionVoltage mAnglePositionVoltage = new PositionVoltage(0);

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    private CANcoderConfigurator angleEncoderConfigurator;
    //private CANcoderConfiguration angleEncoderConfiguration;
    final TrapezoidProfile m_profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(8, 16)
    );
    TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private TalonFXConfigurator mDriveConfigurator;
    private TalonFXConfigurator mAngleConfigurator;
    private MotorOutputConfigs mDriveOutputConfigs = new MotorOutputConfigs();
    private MotorOutputConfigs mAngleOutputConfigs = new MotorOutputConfigs();
    private CANcoderConfiguration mCanCoderConfigs = new CANcoderConfiguration();
    private GenericEntry current;

    //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(driveKS, driveKV, driveKA); 
    //PIDController angleContoller = new PIDController(1, 0, 0);
    
    
    private double absAngle = 0;

    /* Simulation models (null on a real robot; only built when RobotBase.isSimulation()). */
    private static final DCMotor kSimDriveMotor = DCMotor.getKrakenX60(1);
    private static final DCMotor kSimAngleMotor = DCMotor.getKrakenX60(1);
    private static final double kSimDriveMoiKgMetersSq = 0.03;  // ~robot mass reflected to one wheel
    private static final double kSimAngleMoiKgMetersSq = 0.004; // azimuth rotational inertia
    private DCMotorSim mDriveMotorSim;
    private DCMotorSim mAngleMotorSim;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        current = Shuffleboard.getTab("Swerve").add("Current: " + moduleNumber, 0).getEntry();
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "Drive");
        angleEncoderConfigurator = angleEncoder.getConfigurator();
        angleEncoderConfigurator.apply(Robot.ctreConfigs.swerveCanCoderConfig);
        mCanCoderConfigs.MagnetSensor.MagnetOffset = angleOffset.getRotations();
        angleEncoderConfigurator.apply(mCanCoderConfigs);

        
        // configAngleEncoder();

        /* Angle Motor Config */
        //CANivores = "Drive", "Mast"
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "Drive");
        mAngleConfigurator = mAngleMotor.getConfigurator();
        mAngleConfigurator.apply(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleOutputConfigs.Inverted = angleMotorInvert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        mAngleOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        mAngleConfigurator.apply(mAngleOutputConfigs);
        
        //configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "Drive");
        mDriveConfigurator = mDriveMotor.getConfigurator();
        mDriveConfigurator.apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveOutputConfigs.Inverted = moduleConstants.isInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        mDriveOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        mDriveConfigurator.apply(mDriveOutputConfigs);

        //angleEncoder.setPosition(0);

        //
        // configDriveMotor(moduleConstants.isInverted);

        lastAngle = getState().angle;

        // VoltageOut mAngleVoltageOut = new VoltageOut(6);
        // mAngleMotor.setControl(mAngleVoltageOut);
        // 

        resetToAbsolute();

        if (RobotBase.isSimulation()) {
            mDriveMotorSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(kSimDriveMotor, kSimDriveMoiKgMetersSq, driveGearRatio),
                    kSimDriveMotor);
            mAngleMotorSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(kSimAngleMotor, kSimAngleMoiKgMetersSq, angleGearRatio),
                    kSimAngleMotor);

            /* Seed every sim device to a settled, straight-ahead module so the first drive tick
             * doesn't read the resetToAbsolute() seed (which differs per module by angleOffset)
             * and feed CTREModuleState.optimize() a garbage angle. */
            mAngleMotor.getSimState().setRawRotorPosition(0);
            mAngleMotor.getSimState().setRotorVelocity(0);
            mDriveMotor.getSimState().setRawRotorPosition(0);
            mDriveMotor.getSimState().setRotorVelocity(0);
            angleEncoder.getSimState().setRawPosition(-angleOffset.getRotations());
            angleEncoder.getSimState().setVelocity(0);
        }
    }

    /**
     * Simulation only: advances this module's motor physics by {@code dtSeconds} and writes the
     * resulting rotor / CANcoder feedback into the Phoenix sim state, so {@link #getState()},
     * {@link #getPosition()} and {@link #getCanCoder()} report real motion. Driven from
     * {@link Drivetrain#simulationPeriodic()}; a no-op on a real robot.
     */
    public void updateSimState(double dtSeconds){
        if (mDriveMotorSim == null) return;

        TalonFXSimState driveSimState = mDriveMotor.getSimState();
        TalonFXSimState angleSimState = mAngleMotor.getSimState();
        CANcoderSimState encoderSimState = angleEncoder.getSimState();

        double batteryVolts = RobotController.getBatteryVoltage();
        driveSimState.setSupplyVoltage(batteryVolts);
        angleSimState.setSupplyVoltage(batteryVolts);
        encoderSimState.setSupplyVoltage(batteryVolts);

        /* Drive motor: applied voltage -> wheel motion. The rotor runs ahead of the wheel by
         * driveGearRatio (CTREConfigs sets Feedback.SensorToMechanismRatio = driveGearRatio). */
        mDriveMotorSim.setInputVoltage(driveSimState.getMotorVoltage());
        mDriveMotorSim.update(dtSeconds);
        driveSimState.setRawRotorPosition(mDriveMotorSim.getAngularPositionRotations() * driveGearRatio);
        driveSimState.setRotorVelocity(mDriveMotorSim.getAngularVelocityRPM() / 60.0 * driveGearRatio);

        /* Steer motor: applied voltage -> azimuth motion. */
        mAngleMotorSim.setInputVoltage(angleSimState.getMotorVoltage());
        mAngleMotorSim.update(dtSeconds);
        double steerRotations = mAngleMotorSim.getAngularPositionRotations();
        double steerRotationsPerSec = mAngleMotorSim.getAngularVelocityRPM() / 60.0;
        angleSimState.setRawRotorPosition(steerRotations * angleGearRatio);
        angleSimState.setRotorVelocity(steerRotationsPerSec * angleGearRatio);

        /* Drive the CANcoder toward the simulated module angle so getCanCoder() roughly tracks
         * getAngle(). This is best-effort only: the Phoenix CANcoder sim does not resolve
         * getAbsolutePosition() predictably here (the MagnetOffset config appears to race the sim
         * writes), so the sim odometry path uses getAngle() instead -- see
         * Drivetrain.getModulePositionsForOdometry(). This keeps telemetry readouts sane. */
        encoderSimState.setRawPosition(steerRotations - angleOffset.getRotations());
        encoderSimState.setVelocity(steerRotationsPerSec);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        //desiredState.angle = desiredState.angle.plus(Rotation2d.fromRotations(absAngle));
        //desiredState.angle = Rotation2d.fromRotations(absAngle).minus(desiredState.angle);
        desiredState = CTREModuleState.optimize(desiredState, Rotation2d.fromDegrees(mAngleMotor.getPosition().getValueAsDouble() * 360)); 

        //desiredState.speedMetersPerSecond *= desiredState.angle.plus(getCanCoder()).getCos();

        //desiredState.speedMetersPerSecond *= desiredState.angle.minus(getSt)
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / maxSpeed;
            //mDriveMotor.setControl(new DutyCycleOut(percentOutput).withEnableFOC(false));
            mDriveMotor.set(percentOutput);
        }
        else {
            double velocity = (((desiredState.speedMetersPerSecond) / wheelCircumference));
  
            if (velocity == 0) mDriveMotor.setVoltage(0);
            else mDriveMotor.setControl(mDriveVelocityVoltage.withVelocity(velocity));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        //current.setDouble(mAngleMotor.getTorqueCurrent().getValueAsDouble());
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.005)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        
        TrapezoidProfile.State m_goal = new TrapezoidProfile.State(angle.getRotations(), 0);
        m_setpoint = m_profile.calculate(Constants.kTimestepSeconds, m_setpoint, m_goal);
        

        mAngleMotor.setControl(mAnglePositionVoltage.withPosition(m_setpoint.position).withEnableFOC(true));

        lastAngle = angle;
    } 

    private void setAngle(Rotation2d angle)
    {
        mAngleMotor.setControl(mAnglePositionVoltage.withPosition(angle.getRotations()).withEnableFOC(true));
        lastAngle = Rotation2d.fromRotations(absAngle);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(mAngleMotor.getPosition().getValueAsDouble() * 360);
    }

    private Rotation2d getAngleForOdometry() {
        return Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble() % 1);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public TalonFX getDriveMotor(){
        return mDriveMotor;
    }

    public TalonFX getAngleMotor(){
        return mAngleMotor;
    }

    void resetToAbsolute(){
        /* In simulation the steer position is driven entirely by updateSimState(); these blocking
         * config/control calls would just be overwritten next tick, and running four of them on a
         * button press (e.g. the reset-heading combo) blows the 20 ms loop and stutters the sim. */
        if (RobotBase.isSimulation()) return;
        mAngleMotor.setPosition(-getCanCoder().getRotations());
        setAngle(Rotation2d.fromRotations(0));
    }

    private void configAngleEncoder(){        
        // angleEncoder.configFactoryDefault();
        // angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        // mAngleMotor.configFactoryDefault();
        // mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        // mAngleMotor.setInverted(angleMotorInvert);
        // mAngleMotor.setNeutralMode(angleNeutralMode);
        // mDriveConfigurator.apply();
        // resetToAbsolute();
    }

    // private void configDriveMotor(boolean isInverted){        
    //     // mDriveMotor.configFactoryDefault();
    //     // mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
    //     mDriveMotor.setInverted(isInverted);
    //     mDriveMotor.setNeutralMode(driveNeutralMode);
    //     mDriveMotor.setSelectedSensorPosition(0);
    // }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            mDriveMotor.getVelocity().getValueAsDouble() * wheelCircumference, 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            mDriveMotor.getPosition().getValueAsDouble() * wheelCircumference, 
            getAngle()
        );
    }
}