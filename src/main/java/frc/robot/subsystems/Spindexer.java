package frc.robot.subsystems;

// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SpindexerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class Spindexer extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum SpindexerStates {
    StateZero,
    StateFeed,
    StateReverse
  }

  public static SpindexerStates m_spindexerCurrentState;
  public static SpindexerStates m_spindexerRequestedState;

  // set port!!! (ideally port 6 or 7)
  private final TalonFX m_talon = new TalonFX(23, "Other"); 

   private double desiredVoltage = 0;

  public Spindexer() {
    // Misc setup goes here

    var talonFXConfigs = new TalonFXConfiguration();

    // kP outputs 12 volts when the positional error is 12/n rotations
    // kI adds n volts per second when the positional error is 1 rotation
    // kD outputs n volts when the velocity error is 1 rotation per second
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = kPSpindexer;
    slot0Configs.kI = kISpindexer;
    slot0Configs.kD = kDSpindexer;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    if (kSpindexerClockwisePositive) 
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    m_talon.getConfigurator().apply(talonFXConfigs);
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80;// edit later
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 80;// edit later
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_spindexerCurrentState = SpindexerStates.StateZero;
    m_spindexerRequestedState = SpindexerStates.StateZero;
  }

  @Override
  public void periodic() {
    switch (m_spindexerRequestedState) {
      case StateZero:
          desiredVoltage = 0;
          break;
      case StateFeed:
      //remember to change this
          desiredVoltage = 4;
          break;
      case StateReverse:
          desiredVoltage = -4;
          break;

    }
  m_spindexerCurrentState = m_spindexerRequestedState;

  runControlLoop();
  }   

  public void runControlLoop() {
    m_talon.setVoltage(desiredVoltage);
  }
  
  // example of a "setter" method
  public void requestState(SpindexerStates desiredState) {
    m_spindexerRequestedState = desiredState;
  }
 
  // example of a "getter" method
  public SpindexerStates getCurrentState() { 
    return m_spindexerCurrentState; 
  }

}
