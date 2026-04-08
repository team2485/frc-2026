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

//import static frc.robot.Constants.RollerConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterStates;

// Imports go here
import static frc.robot.Constants.FeederConstants.*;

public class Feeder extends SubsystemBase {
    // Misc variables for specific subsystem go here

    // Enum representing all of the states the subsystem can be in
    public enum FeederStates {
        StateOff,
        StateFeeding
    }

    public static FeederStates m_FeederCurrentState;
    public static FeederStates m_FeederRequestedState;

    // You may need more than one motor
    private final TalonFX m_talon = new TalonFX(kFeederPort, "Other");

    private double desiredVelocity = 0;
    private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0).withSlot(0);
    public GenericEntry feederActive;
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
    private RobotContainer m_rc ;
    public Feeder(RobotContainer rc) {
        // Misc setup goes here
        m_rc = rc;
        feederActive = Shuffleboard.getTab("Active").add("Feeder Active", false).getEntry();
        var talonFXConfigs = new TalonFXConfiguration();
        // These will be derived experimentally but in case you are wondering
        // How these terms are defined from the TalonFX docs
        // kS adds n volts to overcome static friction
        // kV outputs n volts when the velocity target is 1 rotation per second
        // kP outputs 12 volts when the positional error is 12/n rotations
        // kI adds n volts per second when the positional error is 1 rotation
        // kD outputs n volts when the velocity error is 1 rotation per second
        var slot0Configs = talonFXConfigs.Slot0;
        
        slot0Configs.kP = kPFeeder;
        slot0Configs.kI = kIFeeder;
        slot0Configs.kD = kDFeeder;
        slot0Configs.kV = kVFeeder;
        slot0Configs.kA = kAFeeder;


        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 200; // Target acceleration of 400 rps/s (0.25 seconds to max)
        // motionMagicConfigs.MotionMagicJerk = 500;



        var motorOutputConfigs = talonFXConfigs.MotorOutput;
        talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        if (kFeederClockwisePositive)
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        else
            motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        
        m_talon.getConfigurator().apply(talonFXConfigs);

        
        m_FeederCurrentState = FeederStates.StateOff;
        m_FeederRequestedState = FeederStates.StateOff;

        // if we design the robot with a proper resting position in mind
        // this should be the only initilization necessary
        // no firstTime2 :)
    }

    @Override
    public void periodic() {
        switch (m_FeederRequestedState) {
            case StateOff:
            feederActive.setBoolean(false);
            if(m_rc.m_shooter.getCurrentState() == ShooterStates.StateShooting){
                desiredVelocity = 0;
            }else{
                desiredVelocity = 0;

                // desiredVelocity = -25;


            }   
                break;
            case StateFeeding:
            feederActive.setBoolean(true);
                if(m_rc.m_shooter.getCurrentState() == ShooterStates.StateShooting){


                    desiredVelocity = 100;

                }
                else{
                    desiredVelocity = 0;

                }
                break;
        }

        runControlLoop();

        // state.setString(m_RollerCurrentState.toString());
        // m_FeederCurrentState.setString(m_FeederRequestedState.toString());
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

        if(desiredVelocity == 0){
            m_talon.setVoltage(0);


        }else{
            m_talon.set(1);


        }
        // m_talonRight.setControl(requestRight);
    }

    // example of a "setter" method
    public void requestState(FeederStates requestedState) {
        m_FeederRequestedState = requestedState;
    }

    // example of a "getter" method
    public FeederStates getCurrentState() {
        return m_FeederCurrentState;
    }
}