package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.drive.CTREConfigs;
import frc.util.CTREModuleState;
import frc.util.Conversions;
import frc.util.CTREModuleState;
import frc.util.SwerveModuleConstants;

import static frc.robot.Constants.Swerve.*;

import frc.robot.Robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


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
        m_setpoint = m_profile.calculate(angleContinuousCurrentLimit, m_setpoint, m_goal);
        

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