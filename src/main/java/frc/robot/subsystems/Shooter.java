package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    // Misc variables for specific subsystem go here

    // Enum representing all of the states the subsystem can be in
    public enum ShooterStates {

        StateOff,
        StateAccelerating,
        StateDeccelerating,
        StateShooting,

    }
    public static final double kShooterTolernace = 20;//rps
    public static ShooterStates m_ShooterCurrentState;
    public static ShooterStates m_ShooterRequestedState;
    public static double accelerationConstant;
    // You may need more than one motor
    private final TalonFX m_talonLeft = new TalonFX(kShooterPortLeft, "Other");
    private final TalonFX m_talonRight = new TalonFX(kShooterPortRight, "Other");

    private double desiredVelocity = 0;
    private final Follower requestRight = (new Follower(22, MotorAlignmentValue.Opposed));
    private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0).withSlot(0);
    public RobotContainer m_rc;
    // private GenericEntry stateLog =
    // Shuffleboard.getTab("Roller").addString("Roller State", "blah").;
    // public static GenericEntry state = Shuffleboard.getTab("Roller").add("State
    // of Roller", "init").getEntry();
    // public static GenericEntry stateRequested =
    // Shuffleboard.getTab("Roller").add("Req. State of Roller", "init").getEntry();
    // public static GenericEntry currentLog =
    // Shuffleboard.getTab("Roller").add("current",0.0).getEntry();
    // public static GenericEntry veloLog =
    // Shuffleboard.getTab("Roller").add("velocity",0.0).getEntry();

    // Unit default for TalonFX libraries is rotations

    public Shooter(RobotContainer rc) {
        // Misc setup goes here
        m_rc=rc;
        var talonFXConfigs = new TalonFXConfiguration();
        // These will be derived experimentally but in case you are wondering
        // How these terms are defined from the TalonFX docs
        // kS adds n volts to overcome static friction
        // kV outputs n volts when the velocity target is 1 rotation per second
        // kP outputs 12 volts when the positional error is 12/n rotations
        // kI adds n volts per second when the positional error is 1 rotation
        // kD outputs n volts when the velocity error is 1 rotation per second
        var slot0Configs = talonFXConfigs.Slot0;
        
        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = kVShooter;
        slot0Configs.kA = kAShooter;


        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000;

        var motorOutputConfigs = talonFXConfigs.MotorOutput;
        talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 80;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 160;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        if (kShooterLeftClockwisePositive)
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        else
            motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        
        m_talonLeft.getConfigurator().apply(talonFXConfigs);

        if (kShooterRightClockwisePositive)
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        else
            motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_talonRight.getConfigurator().apply(talonFXConfigs);

        m_ShooterCurrentState = ShooterStates.StateOff;
        m_ShooterRequestedState = ShooterStates.StateOff;
        
        m_talonLeft.setPosition(0);
        m_talonRight.setPosition(0);
    }

    @Override
    public void periodic() {
        switch (m_ShooterCurrentState) {
            case StateOff:
                desiredVelocity = 0;
                break;
            case StateAccelerating:
                double dist = m_rc.tracker.getAimPose().getTranslation().getDistance(m_rc.m_drivetrain.getState().Pose.getTranslation());
                if(dist > 4){
                    desiredVelocity = 80;
                    
                }
                else if(dist >3){

                    desiredVelocity = 65;


                
                }else{
                    desiredVelocity = 50;

                }
                m_ShooterRequestedState = ShooterStates.StateShooting;
                break;
            case StateDeccelerating:
                desiredVelocity = 0;
                m_ShooterRequestedState = ShooterStates.StateOff;
                break;
            case StateShooting:
                double dist2 = m_rc.tracker.getAimPose().getTranslation().getDistance(m_rc.m_drivetrain.getState().Pose.getTranslation());
                if(dist2 > 4){
                    desiredVelocity = 85;
                    
                }
                else if(dist2 >3){

                    desiredVelocity = 65;


                
                }else{
                    desiredVelocity = 50;

                }
                break;
        }
        if(m_ShooterRequestedState == ShooterStates.StateAccelerating || m_ShooterRequestedState == ShooterStates.StateDeccelerating){

            m_ShooterCurrentState = m_ShooterRequestedState;

        }
        runControlLoop();

        // state.setString(m_RollerCurrentState.toString());
        // m_ShooterCurrentState.setString(m_ShooterRequestedState.toString());
    }
    
    // public boolean isStalling(){

    // if(m_talon.getVelocity().getValueAsDouble() < 1.5 &&
    // m_talon.getSupplyCurrent().getValueAsDouble() > 10 ){

    // return true;

    // }
    // return false;

    // }

    public void runControlLoop() {
        // currentLog.setDouble(m_talon.getSupplyCurrent().getValueAsDouble());
        // veloLog.setDouble(m_talon.getVelocity().getValueAsDouble());
        if(m_talonLeft.getDeviceTemp().getValueAsDouble() > 50){

            m_talonLeft.set(0);
            m_talonRight.set(0);
            return;

        }
        if(desiredVelocity == 0){
            m_talonLeft.setVoltage(0);
            m_talonRight.setVoltage(0);


        }else{
            m_talonLeft.setControl(request.withVelocity(desiredVelocity));
            m_talonRight.setControl(requestRight);  
        }
        if(m_ShooterRequestedState == ShooterStates.StateShooting){
            if(Math.abs(m_talonLeft.getVelocity().getValueAsDouble() - desiredVelocity) < kShooterTolernace){ // tolerance
                
                m_ShooterCurrentState = ShooterStates.StateShooting;

            }
            else{
                m_ShooterCurrentState = ShooterStates.StateAccelerating;

            }

        }

        if(m_ShooterRequestedState == ShooterStates.StateOff) m_ShooterCurrentState = m_ShooterRequestedState;



        
    }

    // example of a "setter" method
    public void requestState(ShooterStates requestedState) {
        m_ShooterRequestedState = requestedState;
    }

    // example of a "getter" method
    public ShooterStates getCurrentState() {
        return m_ShooterCurrentState;
    }

    public double getDesiredVelocity(){
        return desiredVelocity;
    }

    public double getVelocity() {
        return m_talonLeft.getVelocity().getValueAsDouble();
    } 

    // misc methods go here, getters and setters should follow above format
}