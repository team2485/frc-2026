package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import static frc.robot.Constants.RollerConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.DistanceLookup;

// Imports go here
import static frc.robot.Constants.AnglerConstants.*;


public class Angler extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum AnglerStates {
    StateZero,
    StateMid,
    StateMax,
    StateOpening,
    StateClosing,
    StateTest4,
    StateTest3,
    StateTest2,
    StateTest1,
    StateAuto
  }

  public static AnglerStates m_AnglerCurrentState;
  public static AnglerStates m_AnglerRequestedState;
  public static final DistanceLookup lookupTable = new DistanceLookup();
  private final TalonFX m_talon = new TalonFX(kAnglerPort,"Other");
  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
  // private GenericEntry stateLog = Shuffleboard.getTab("Roller").addString("Roller State", "blah").;
  // public static GenericEntry state = Shuffleboard.getTab("Roller").add("State of Roller", "init").getEntry();
  // public static GenericEntry stateRequested = Shuffleboard.getTab("Roller").add("Req. State of Roller", "init").getEntry();
  // public static GenericEntry currentLog = Shuffleboard.getTab("Roller").add("current",0.0).getEntry();
  // public static GenericEntry veloLog = Shuffleboard.getTab("Roller").add("velocity",0.0).getEntry();

  // Unit default for TalonFX libraries is rotations
   private double desiredPosition = 0;

  public Angler() {
    // Misc setup goes here
    var talonFXConfigs = new TalonFXConfiguration();
    // These will be derived experimentally but in case you are wondering
    // How these terms are defined from the TalonFX docs
    // kS adds n volts to overcome static friction
    // kV outputs n volts when the velocity target is 1 rotation per second
    // kP outputs 12 volts when the positional error is 12/n rotations
    // kI adds n volts per second when the positional error is 1 rotation
    // kD outputs n volts when the velocity error is 1 rotation per second
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = kPAngler;
    slot0Configs.kI = kIAngler;
    slot0Configs.kD = kDAngler;
    slot0Configs.kV = kVAngler;
    slot0Configs.kA = kAAngler;


    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 50; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicCruiseVelocity = 5;
    motionMagicConfigs.MotionMagicJerk = 500;

    

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable=true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit=40;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit=60;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable=true;
    if (kAnglerClockwisePositive)
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    m_talon.getConfigurator().apply(talonFXConfigs);

    m_AnglerCurrentState = AnglerStates.StateZero;
    m_AnglerRequestedState = AnglerStates.StateZero;

    m_talon.setPosition(0);
  }

  @Override
  public void periodic() {
    switch (m_AnglerRequestedState) {
      case StateZero:
        desiredPosition = 0.01;
        break;
      case StateAuto:

    
        desiredPosition = lookupTable.getValue(0.1);
        break;
      case StateMid:
        desiredPosition = .2;
        break;
      case StateMax:
        desiredPosition = 1; // This hits @ 4.648 metres
        break;
      case StateTest1: // This hits at 50 inches from centre (1.27M)
        desiredPosition = .2;
        break;

      case StateTest2:
        desiredPosition = .3;
        break;
      case StateTest3:
        desiredPosition = .4;
        break;
      case StateTest4:
        desiredPosition = .5;
        break;
      case StateOpening:
        if(desiredPosition < 6)
        {
          desiredPosition+=0.05;
        }
        break;
      case StateClosing:
         if(desiredPosition > 0)
        {
          desiredPosition-=0.05;
        }
        break;
      }
      desiredPosition*=23; // gear ratio
 
    runControlLoop();

    // state.setString(m_RollerCurrentState.toString());
    //m_AnglerCurrentState.setString(m_AnglerRequestedState.toString());
  }

  // public boolean isStalling(){

  //   if(m_talon.getVelocity().getValueAsDouble() < 1.5 && m_talon.getSupplyCurrent().getValueAsDouble() > 10 ){

  //     return true;

  //   }
  //   return false;

  // }

  public void runControlLoop() {
    //currentLog.setDouble(m_talon.getSupplyCurrent().getValueAsDouble());
    //veloLog.setDouble(m_talon.getVelocity().getValueAsDouble());
   

    m_talon.setControl(request.withPosition(desiredPosition));
  }

  // example of a "setter" method
  public void requestState(AnglerStates requestedState) {
    m_AnglerRequestedState = requestedState;
  }
 
  // example of a "getter" method
  public AnglerStates getCurrentState() {
    return m_AnglerCurrentState;
  }

  // misc methods go here, getters and setters should follow above format
}