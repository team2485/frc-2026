package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import javax.print.attribute.standard.MediaSize.Other;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicDutyCycle;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.Constants.IntakeConstants.*;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.PoseEstimation;
import frc.robot.subsystems.drive.TargetTracking.TargetingStates;

import com.ctre.phoenix6.controls.Follower;

public class Intake extends SubsystemBase {
    private IntakeStates m_IntakeRequestedState;
    private IntakeStates m_IntakeCurrentState;
    private final TalonFX m_talon_Roll = new TalonFX(26, "Other");
    // private final TalonFX m_talon_L = new TalonFX(25, "Other");
    private final TalonFX m_talon_winchR = new TalonFX(27, "Other");
    private final TalonFX m_talon_winchL = new TalonFX(24, "Other"); // change so that it's wired to Other canbus later
    private double desiredRollerVelocity = 0;
    private double desiredWinchVelocity = 0;
    private boolean extendedIntake = false;
    private final MotionMagicVelocityVoltage velRollerRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
    // private final PositionVoltage posWinchRequestL = new PositionVoltage(0).withSlot(0);
    // private final PositionVoltage posWinchRequestR = new PositionVoltage(0).withSlot(0);
    // private final PositionDutyCycle posWinchRequestL = new PositionDutyCycle(0).withSlot(0);
    // private final PositionDutyCycle posWinchRequestR = new PositionDutyCycle(0).withSlot(0);
    // private final MotionMagicVelocityVoltage posWinchRequestL = new MotionMagicVelocityVoltage(0).withSlot(0);
    // private final MotionMagicVelocityVoltage posWinchRequestR = new MotionMagicVelocityVoltage(0).withSlot(0);
    private final DynamicMotionMagicDutyCycle posWinchRequestL = new DynamicMotionMagicDutyCycle(0, 0, 0);
    private final DynamicMotionMagicDutyCycle posWinchRequestR = new DynamicMotionMagicDutyCycle(0, 0, 0);
    private final MotionMagicVelocityVoltage velWinchRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
    private final MotionMagicVelocityVoltage velWinchRequestR = new MotionMagicVelocityVoltage(0).withSlot(0);
    private double desiredPosition = 0;
    // private final Follower RRollerFollower = new Follower(26,
    // MotorAlignmentValue.Opposed);
    // private final Follower RIntakePinion = new Follower(27,
    // MotorAlignmentValue.Opposed);

    public static GenericEntry velocityL;
    public static GenericEntry velocityR;
    public static GenericEntry voltageL;
    public static GenericEntry voltageR;

    public enum IntakeStates {
        StateStartup, // Immediately after enabling, pull in winch until current spike, zero it, then
                      // go to StateRetracted
        StateRetracted, // Intake Retracted
        StateIntaking, // Extended and running
        StateOuttaking, // Extended and running backwards
        StateIdle, // Extended, but not running
        StateSlowRetract,
        StateExtendedIdlingWheels,
        StateRetracting,
        StateShooting
    }

    public Intake() {
        velocityL = Shuffleboard.getTab("Intake").add("Left Velocity", 0.0).getEntry();
        velocityR = Shuffleboard.getTab("Intake").add("Right Velocity", 0.0).getEntry();
        voltageR = Shuffleboard.getTab("Intake").add("Right Voltage", 0.0).getEntry();
        voltageL = Shuffleboard.getTab("Intake").add("Left Voltage", 0.0).getEntry();

        var talonFXConfigs = new TalonFXConfiguration();
        // These will be derived experimentally but in case you are wondering
        // How these terms are defined from the TalonFX docs
        // kS adds n volts to overcome static friction
        // kV outputs n volts when the velocity target is 1 rotation per second
        // kP outputs 12 volts when the positional error is 12/n rotations
        // kI adds n volts per second when the positional error is 1 rotation
        // kD outputs n volts when the velocity error is 1 rotation per second
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = kPIntakeRoller;
        slot0Configs.kI = kIIntakeRoller;
        slot0Configs.kD = kDIntakeRoller;
        slot0Configs.kV = kVIntakeRoller;
        slot0Configs.kA = kAIntakeRoller;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80;
        motionMagicConfigs.MotionMagicAcceleration = 45;
        // motionMagicConfigs.MotionMagicJerk = 500;

        var motorOutputConfigs = talonFXConfigs.MotorOutput;
        talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 50;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 60;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        if (kIntakeClockwisePositive) {
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        }
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        m_talon_Roll.getConfigurator().apply(talonFXConfigs);
        // m_talon_L.getConfigurator().apply(talonFXConfigs);

        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 50;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80;
        slot0Configs.kP = kPIntakeWinch;
        slot0Configs.kI = kIIntakeWinch;
        slot0Configs.kD = kDIntakeWinch;
        slot0Configs.kV = kVIntakeWinch;
        slot0Configs.kA = kAIntakeWinch;
        slot0Configs.kS = 1;
        motionMagicConfigs.MotionMagicCruiseVelocity = 50;
        motionMagicConfigs.MotionMagicAcceleration = 30;
        // motionMagicConfigs.MotionMagicJerk = 500;

        if (kWinchClockwisePositive) {
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        m_talon_winchR.getConfigurator().apply(talonFXConfigs);

        if (kWinchClockwisePositive) {
            motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        } else {
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        }

        // motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        m_talon_winchL.getConfigurator().apply(talonFXConfigs);
        m_talon_winchR.setPosition(0);
        m_talon_winchL.setPosition(0);

        m_IntakeCurrentState = IntakeStates.StateRetracted;
        m_IntakeRequestedState = IntakeStates.StateRetracted;
    }

    boolean RReeled = false;
    boolean LReeled = false;

    // int time = 0;
    boolean slow = false;

    @Override
    public void periodic() {
        // if (DriverStation.isEnabled())
        // time++;
        velocityL.setDouble(m_talon_winchL.getClosedLoopError().getValueAsDouble());
        velocityR.setDouble(m_talon_winchR.getClosedLoopError().getValueAsDouble());
        voltageL.setDouble(m_talon_winchL.getMotorVoltage().getValueAsDouble());
        voltageR.setDouble(m_talon_winchR.getMotorVoltage().getValueAsDouble());
        // velocityL.setDouble(m_talon_winchL.);
        switch (m_IntakeCurrentState) {
            case StateStartup:
                desiredWinchVelocity = -1;
                // if (Math.abs(m_talon_winchR.getStatorCurrent().getValueAsDouble()) > 30 &&
                // time > 40) {
                // // desiredWinchVelocity = 0;
                m_talon_winchR.setPosition(0);
                RReeled = true;
                // }
                // if (Math.abs(m_talon_winchL.getStatorCurrent().getValueAsDouble()) > 20 &&
                // time > 40) {
                // desiredWinchVelocity = 0;
                m_talon_winchL.setPosition(0);
                LReeled = true;
                // }
                if (RReeled && LReeled) {
                    m_IntakeCurrentState = IntakeStates.StateRetracted;
                    m_IntakeRequestedState = IntakeStates.StateRetracted;
                    desiredWinchVelocity = 0;
                }
                break;
            case StateExtendedIdlingWheels:
                extendedIntake = true;
                desiredRollerVelocity=0;
                break;
            case StateSlowRetract:
                desiredWinchVelocity = 0;
                desiredRollerVelocity = 0;
                slow = true;
                extendedIntake = false;
                break;
            case StateRetracting:
                extendedIntake = false;
                slow = false;
                desiredRollerVelocity = 80;
                desiredWinchVelocity = 0;
                break;
            case StateRetracted:
                extendedIntake = false;
                slow = false;
                desiredRollerVelocity = 0;
                desiredWinchVelocity = 0;
                break;
            case StateIntaking:
                extendedIntake = true;
                slow = false;
                desiredWinchVelocity = 0;
                desiredRollerVelocity = 80;
                break;
            case StateShooting:
                // extendedIntake = false;
                // slow=false; 
                desiredRollerVelocity = 0;
                break;
            case StateOuttaking:
                extendedIntake = true;
                desiredWinchVelocity = 0;
                desiredRollerVelocity = -50;
                break;

            case StateIdle:
                extendedIntake = true;
                desiredRollerVelocity = 0;
                break;
        }

        stateSwitchLogic();
        runControlLoop();
        // System.out.println("Intake State: " + m_IntakeCurrentState);
        // System.out.println("RM: " +
        // m_talon_winchR.getStatorCurrent().getValueAsDouble() + " LM: "
        // + m_talon_winchL.getStatorCurrent().getValueAsDouble());
        // System.out.println("Time: " + time);
    }

    boolean lastExtended = false;

    public void runControlLoop() {
        // m_talon_L.setControl(RRollerFollower);
        // ^set the control of the left intake motor to that of the Follower opposing
        // the direction of the right intake motor
        if (desiredRollerVelocity == 0) {
            m_talon_Roll.setVoltage(0);
            // m_talon_L.setVoltage(0);
        }
        else {
            m_talon_Roll.setControl(velRollerRequest.withVelocity(desiredRollerVelocity));
        }
        // m_talon_winchL.setControl(new NeutralOut());
        if (desiredWinchVelocity != 0) {
            m_talon_winchR.setControl(velWinchRequestR.withVelocity(desiredWinchVelocity).withAcceleration(1));
            m_talon_winchL.setControl(velWinchRequest.withVelocity(desiredWinchVelocity).withAcceleration(1));
        } else {
            // if(m_talon_L.getProcessorTemp().getValueAsDouble() >= 60){

            // System.err.println("LEFT MOTOR OVERHEAT!!");
            // m_talon_L.setControl(new NeutralOut());
            // return;
            // }
            if (m_talon_Roll.getDeviceTemp().getValueAsDouble() >= 60) {
                System.err.println("ROLLER MOTOR OVERHEAT!!");

                m_talon_Roll.setControl(new NeutralOut());

                return;
            }
            if (m_talon_winchR.getDeviceTemp().getValueAsDouble() >= 60) {
                System.err.println("RIGHT MOTOR OVERHEAT!!");

                m_talon_winchR.setControl(new NeutralOut());

                return;
            }
            if (m_talon_winchL.getDeviceTemp().getValueAsDouble() >= 60) {
                System.err.println("LEFT MOTOR OVERHEAT!!");

                m_talon_winchL.setControl(new NeutralOut());

                return;
            }

            if (extendedIntake) {
                // m_talon_winchL.setControl(posWinchRequestL.withPosition(desiredPosition));
                // m_talon_winchR.setControl(posWinchRequestR.withPosition(5));
                desiredPosition = (-18 * 3)*24/23;
            } else if (m_IntakeCurrentState != IntakeStates.StateStartup) {
                desiredPosition = .1;
                // m_talon_winchL.setControl(posWinchRequestL.withPosition(0.1));
                // m_talon_winchR.setControl(posWinchRequestR.withPosition(0.1));
            }
            posWinchRequestL.Position = desiredPosition;
            posWinchRequestR.Position = desiredPosition;
            if (extendedIntake != lastExtended) {
                if (slow) {
                    m_talon_winchL.setControl(posWinchRequestL.withVelocity(15).withAcceleration(10));
                    m_talon_winchR.setControl(posWinchRequestR.withVelocity(15).withAcceleration(10));
                } else {
                    m_talon_winchL.setControl(posWinchRequestL.withVelocity(50).withAcceleration(40));
                    m_talon_winchR.setControl(posWinchRequestR.withVelocity(50).withAcceleration(40));
                    // m_talon_Roll.setControl(velRollerRequest.withVelocity(desiredRollerVelocity));
                }
            }

            lastExtended = extendedIntake;
        }
    }

    public void stateSwitchLogic() {
        m_IntakeCurrentState = m_IntakeRequestedState; // Put the checks for switching states here :)
    }

    public void requestState(IntakeStates req) {
        m_IntakeRequestedState = req;
    }

    public IntakeStates getState() {
        return m_IntakeCurrentState;
    }

    public IntakeStates getRequestedState() {
        return m_IntakeRequestedState;
    }
}
