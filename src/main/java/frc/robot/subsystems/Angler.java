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
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.PoseEstimation;
import frc.robot.subsystems.drive.TargetTracking;
import frc.util.DistanceLookup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

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
    StateHub,
    StateTest3,
    StateTest2,
    StateTest1,
    StateAuto,
    StateAmplify
  }

  public static AnglerStates m_AnglerCurrentState;
  public static AnglerStates m_AnglerRequestedState;
  public static Drivetrain m_drivetrain;
  public static TargetTracking m_TargetTracking;
  public RobotContainer m_robotContainer;
  public static final DistanceLookup lookupTable = DistanceLookup.getSelf();
  private final TalonFX m_talon = new TalonFX(kAnglerPort, "Other");
  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
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
  private double desiredPosition = 0;
  public Shooter m_shooter ;//= new Shooter();

  public Angler(Drivetrain drive, TargetTracking tt, RobotContainer rc,Shooter shooter) {
    m_shooter=shooter;
    m_drivetrain = drive;
    m_TargetTracking = tt;
    m_robotContainer = rc;
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
    motionMagicConfigs.MotionMagicAcceleration = 175; // Target acceleration of 400 rps/s (0.25 seconds to max) //earlier int was at 90
    motionMagicConfigs.MotionMagicCruiseVelocity = 150; // 100 earlier
    motionMagicConfigs.MotionMagicJerk = 1250; //750 earlier

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 20;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 40;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    if (kAnglerClockwisePositive)
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else
      motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    m_talon.getConfigurator().apply(talonFXConfigs);

    m_AnglerCurrentState = AnglerStates.StateZero;
    m_AnglerRequestedState = AnglerStates.StateZero;

    m_talon.setPosition(0);
  }
  double rangeBoost = 0;
  boolean increased = false;

  @Override
  public void periodic() {
    switch (m_AnglerRequestedState) {
      case StateZero:
        desiredPosition = 0.001;
        break;
      case StateAuto:
        // m_drivetrain.getState().Pose.getTranslation()
        double dist = m_TargetTracking.getAimPose().getTranslation().getDistance(m_drivetrain.getState().Pose.getTranslation());
        desiredPosition = lookupTable.getValue(dist ) +rangeBoost;
        break;
       case StateMid:
        desiredPosition = .5;
        break;
      case StateMax:
        desiredPosition = 7.0/9.0; // This hits @ 4.648 metres
        break;
      case StateTest1: // This hits at 50 inches from centre (1.27M)
        desiredPosition = .3;
        break;

      case StateTest2:
        desiredPosition = .67;
        break;
      case StateTest3:
        desiredPosition = .5;
        break;
      case StateHub:
        desiredPosition = .05;
        break;
      case StateOpening:
        if (desiredPosition < 6) {
          desiredPosition += 0.05;
        }
        break;
      case StateClosing:
        if (desiredPosition > 0) {
          desiredPosition -= 0.05;
        }
        break;
      case StateAmplify:
        // increased = !increased;
        rangeBoost+=.01;
        m_AnglerRequestedState = AnglerStates.StateAuto;
        break;
    }
    if (increased) {
      // desiredPosition += 0.03;
    }
    // System.out.println("Hood: " + desiredPosition);
    desiredPosition *= 9; // gear ratio
    // desiredPosition += 3 * ( m_shooter.getDesiredVelocity() -  m_shooter.getVelocity()) * (1/173); // what is this

    runControlLoop();


    //.005 rot from 100 to 80

    // state.setString(m_RollerCurrentState.toString());
    // m_AnglerCurrentState.setString(m_AnglerRequestedState.toString());
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
    if(m_AnglerCurrentState == AnglerStates.StateZero && m_talon.getVelocity().getValueAsDouble() < 1){

      m_talon.setVoltage( 0); // Setting voltage to zero if we're at zero position for battery conservation....

    }
    // else if(m_AnglerCurrentState == AnglerStates.StateAuto && !(m_robotContainer.m_shooter.getCurrentState() == ShooterStates.StateAccelerating ||m_robotContainer.m_shooter.getCurrentState() == ShooterStates.StateShooting) ){

    //   m_talon.setVoltage( 0);

    // }else{
      m_talon.setControl(request.withPosition(desiredPosition));

    // }
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