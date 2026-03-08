package frc.robot.subsystems;

// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterStates;

import static frc.robot.Constants.SpindexerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.math.controller.PIDController;

public class Spindexer extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum SpindexerStates {
    StateZero,
    StateFeed,
    StateReverse,
    StateAutomatedEnable,
    StateAutomatedOff
  }

  private double desiredVelocity = 0;
   private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0).withSlot(0);

  public static SpindexerStates m_spindexerCurrentState;
  public static SpindexerStates m_spindexerRequestedState;

  private final TalonFX m_talon = new TalonFX(23, "Other"); 
  public RobotContainer m_RobotContainer;
  public Spindexer(RobotContainer c) {
    m_RobotContainer=c;
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

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 50; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 500;
    if (kSpindexerClockwisePositive) 
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    m_talon.getConfigurator().apply(talonFXConfigs);
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 100;// edit later
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
          desiredVelocity = 0;
          break;
      case StateFeed:
          // remember to change this
          desiredVelocity = 35;
          break;
      case StateReverse:
          desiredVelocity = -25;
          break;
      case StateAutomatedEnable:
          if(m_RobotContainer.m_shooter.getCurrentState() == ShooterStates.StateShooting ){

            desiredVelocity = 35;


          }
          else{
            // desiredVelocity =0 ;

          }

          break;
      case StateAutomatedOff:
          //desiredVelocity = 0;
          break;
      default:
      break;
    }
  if(m_spindexerRequestedState == SpindexerStates.StateAutomatedEnable || m_spindexerRequestedState == SpindexerStates.StateAutomatedOff){
    if(m_spindexerCurrentState == SpindexerStates.StateZero || m_spindexerRequestedState == SpindexerStates.StateAutomatedEnable || m_spindexerRequestedState == SpindexerStates.StateAutomatedOff){

      m_spindexerCurrentState = m_spindexerRequestedState;
      

    }


  }else{
    m_spindexerCurrentState = m_spindexerRequestedState;


  }

  runControlLoop();
  }   
  int timer = 0;
  public void runControlLoop() {
    timer ++;
  if(desiredVelocity == 0){
      m_talon.setVoltage(0);
  }
  else{
    if(timer >50 && timer <60){
      // m_talon.setControl(request.withVelocity(-desiredVelocity));
      m_talon.set(0);
    }
    else if(timer > 60){

      m_talon.set(-0.5);

    }
    else{
      // m_talon.setControl(request.withVelocity(desiredVelocity));
      m_talon.set(.5);

    }
    
    if(timer >70){
     timer=0;

    }
  }

  if (m_talon.getDeviceTemp().getValueAsDouble() >= 60) {
                System.out.println("SPINDEXER OVERHEAT!!");

                m_talon.setControl(new NeutralOut());

                return;
            }
  }
  
  // example of a "setter" method
  public void requestState(SpindexerStates desiredState) {
    if(desiredState == SpindexerStates.StateAutomatedEnable || desiredState == SpindexerStates.StateAutomatedOff){
      if(m_spindexerCurrentState == SpindexerStates.StateZero || m_spindexerRequestedState == SpindexerStates.StateAutomatedEnable || m_spindexerRequestedState == SpindexerStates.StateAutomatedOff){

        m_spindexerRequestedState = desiredState;
        

      }


  }else{
    m_spindexerRequestedState = desiredState;


  }

    // m_spindexerRequestedState = desiredState;
  }
 
  // example of a "getter" method
  public SpindexerStates getCurrentState() { 
    return m_spindexerCurrentState; 
  }

}
