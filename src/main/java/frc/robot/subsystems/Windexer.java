package frc.robot.subsystems;

// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterStates;

import static frc.robot.Constants.WindexerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.math.controller.PIDController;

public class Windexer extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum WindexerStates {
    StateZero,
    StateFeed,
    StateReverse,
    StateAutomatedEnable,
    StateAutomatedOff
  }

  private double desiredVelocity = 0;
   private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0).withSlot(0);

  public static WindexerStates m_windexerCurrentState;
  public static WindexerStates m_windexerRequestedState;

  private final TalonFX m_talon = new TalonFX(31, "Other"); 
  public RobotContainer m_RobotContainer;
  public Windexer(RobotContainer c) {
    m_RobotContainer=c;
    // Misc setup goes here

    var talonFXConfigs = new TalonFXConfiguration();

    // kP outputs 12 volts when the positional error is 12/n rotations
    // kI adds n volts per second when the positional error is 1 rotation
    // kD outputs n volts when the velocity error is 1 rotation per second
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = kPWindexer;
    slot0Configs.kI = kIWindexer;
    slot0Configs.kD = kDWindexer;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 50; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 500;
    if (kWindexerClockwisePositive) 
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 50;// edit later
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 50;// edit later
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_talon.getConfigurator().apply(talonFXConfigs);

    m_windexerCurrentState = WindexerStates.StateZero;
    m_windexerRequestedState = WindexerStates.StateZero;
  }

  @Override
  public void periodic() {
    switch (m_windexerRequestedState) {
      case StateZero:
          desiredVelocity = 0;
          break;
      case StateFeed:
          desiredVelocity = 70;
          break;
      case StateReverse:
          desiredVelocity = -25;
          break;
      case StateAutomatedEnable:
          if(m_RobotContainer.m_shooter.getCurrentState() == ShooterStates.StateShooting) {
              desiredVelocity = 70;
          }
          else{
            desiredVelocity = 0;
          }
          break;
      case StateAutomatedOff:
          desiredVelocity = 0;
          break;
      default:
      break;
    }
  if(m_windexerRequestedState == WindexerStates.StateAutomatedEnable || m_windexerRequestedState == WindexerStates.StateAutomatedOff){
    if(m_windexerCurrentState == WindexerStates.StateZero || m_windexerRequestedState == WindexerStates.StateAutomatedEnable || m_windexerRequestedState == WindexerStates.StateAutomatedOff){

      m_windexerCurrentState = m_windexerRequestedState;
      
    }


  }else{
    m_windexerCurrentState = m_windexerRequestedState;


  }

  runControlLoop();
  }   
  public void runControlLoop() {
      m_talon.set(desiredVelocity/100);
      if (m_talon.getDeviceTemp().getValueAsDouble() >= 60) {
            System.err.println("SPINDEXER OVERHEAT!!");

            m_talon.setControl(new NeutralOut());

            return;
        }
  }
  
  // example of a "setter" method
  public void requestState(WindexerStates desiredState) {
    if(desiredState == WindexerStates.StateAutomatedEnable || desiredState == WindexerStates.StateAutomatedOff){
      if(m_windexerCurrentState == WindexerStates.StateZero || m_windexerRequestedState == WindexerStates.StateAutomatedEnable || m_windexerRequestedState == WindexerStates.StateAutomatedOff){

        m_windexerRequestedState = desiredState;
        

      }


  }else{
    m_windexerRequestedState = desiredState;


  }

    // m_windexerRequestedState = desiredState;
  }
 
  // example of a "getter" method
  public WindexerStates getCurrentState() { 
    return m_windexerCurrentState; 
  }

}
