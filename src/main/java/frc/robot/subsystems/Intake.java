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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
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
    private final TalonFX m_talon_R = new TalonFX(26, "Other");
    private final TalonFX m_talon_L = new TalonFX(25, "Other");
    private final TalonFX m_talon_winchR = new TalonFX(24, "Other");
    private final TalonFX m_talon_winchL = new TalonFX(27, "Drive"); // change so that it's wired to Other canbus later
    private double desiredRollerVelocity = 0;
    private double desiredWinchVelocity = 0;
    private boolean extendedIntake = false;
    private final MotionMagicVelocityVoltage velRollerRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
    private final PositionVoltage posWinchRequestL = new PositionVoltage(0).withSlot(0);
    private final PositionVoltage posWinchRequestR = new PositionVoltage(0).withSlot(0);
    private final MotionMagicVelocityVoltage velWinchRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
    private final MotionMagicVelocityVoltage velWinchRequestR = new MotionMagicVelocityVoltage(0).withSlot(0);

    private final Follower RRollerFollower = new Follower(26, MotorAlignmentValue.Opposed);

    public enum IntakeStates {
        StateStartup, // Immediately after enabling, pull in winch until current spike, zero it, then go to StateRetracted
        StateRetracted, // Intake Retracted
        StateIntaking, // Extended and running
        StateOuttaking, // Extended and running backwards
        StateIdle, // Extended, but not running
    }

    public Intake() {
        var talonFXConfigs = new TalonFXConfiguration();
        // These will be derived experimentally but in case you are wondering
        // How these terms are defined from the TalonFX docs
        // kS adds n volts to overcome static friction
        // kV outputs n volts when the velocity target is 1 rotation per second
        // kP outputs 12 volts when the positional error is 12/n rotations
        // kI adds n volts per second when the positional error is 1 rotation
        // kD outputs n volts when the velocity error is 1 rotation per second
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 5;
        slot0Configs.kI = 1;
        slot0Configs.kD = .2;
        slot0Configs.kV = kVIntakeRoller;
        slot0Configs.kA = 0.12;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 35;
        motionMagicConfigs.MotionMagicAcceleration = 25;
        // motionMagicConfigs.MotionMagicJerk = 500;

        var motorOutputConfigs = talonFXConfigs.MotorOutput;
        talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 50;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        if (kIntakeClockwisePositive) {
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        }
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        m_talon_R.getConfigurator().apply(talonFXConfigs);
        m_talon_L.getConfigurator().apply(talonFXConfigs);

        slot0Configs.kP = .5;
        slot0Configs.kI = kIIntakeWinch;
        slot0Configs.kD = .1;
        slot0Configs.kV = 1;
        slot0Configs.kA = .3;

        motionMagicConfigs.MotionMagicCruiseVelocity = 2.5;
        motionMagicConfigs.MotionMagicAcceleration = 1;
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

        m_talon_winchL.getConfigurator().apply(talonFXConfigs);

        m_IntakeCurrentState = IntakeStates.StateStartup;
        m_IntakeRequestedState = IntakeStates.StateStartup;
    }

    boolean RReeled = false;
    boolean LReeled = false;

    int time = 0;

    @Override
    public void periodic() {
        if (DriverStation.isEnabled())
            time++;
        switch (m_IntakeCurrentState) {
            case StateStartup:
                desiredWinchVelocity = -10;
                // desiredWinchVelocity = -10;
                if (m_talon_winchR.getStatorCurrent().getValueAsDouble() > 48 && time > 40) {
                    desiredWinchVelocity = 0;
                    m_talon_winchR.setPosition(0);
                    RReeled = true;
                }
                if (m_talon_winchL.getStatorCurrent().getValueAsDouble() > 48 && time > 40) {
                    desiredWinchVelocity = 0;
                    m_talon_winchL.setPosition(0);
                    LReeled = true;
                }
                if (RReeled && LReeled) {
                    m_IntakeCurrentState = IntakeStates.StateRetracted;
                    m_IntakeRequestedState = IntakeStates.StateRetracted;

                }
                break;

            case StateRetracted:
                extendedIntake = false;
                desiredRollerVelocity = 0;
                desiredWinchVelocity = 0;

                break;

            case StateIntaking:
                extendedIntake = true;
                desiredRollerVelocity = 35;
                break;

            case StateOuttaking:
                extendedIntake = true;
                desiredRollerVelocity = -35;
                break;

            case StateIdle:
                extendedIntake = true;
                desiredRollerVelocity = 0;
                break;
        }

        stateSwitchLogic();
        runControlLoop();
        // System.out.println("Intake State: " + m_IntakeCurrentState);
        // System.out.println("RM: " + m_talon_winchR.getStatorCurrent().getValueAsDouble() + " LM: "
                // + m_talon_winchL.getStatorCurrent().getValueAsDouble());
        // System.out.println("Time: " + time);
    }

    public void runControlLoop() {
        m_talon_R.setControl(velRollerRequest.withVelocity(desiredRollerVelocity).withLimitReverseMotion(true)
                .withEnableFOC(true));
        m_talon_L.setControl(RRollerFollower);
        // ^set the control of the left intake motor to that of the Follower opposing the direction of the right intake motor
        if(desiredRollerVelocity == 0)
        {
            m_talon_R.setVoltage(0);
            m_talon_L.setVoltage(0);
        }
        if (desiredWinchVelocity != 0) {
            m_talon_winchR.setControl(velWinchRequestR.withVelocity(desiredWinchVelocity).withAcceleration(1));
            m_talon_winchL.setControl(velWinchRequest.withVelocity(desiredWinchVelocity).withAcceleration(1));
        } else {
            if (extendedIntake) {
                m_talon_winchR.setControl(posWinchRequestR.withPosition(12));
                m_talon_winchL.setControl(posWinchRequestL.withPosition(12));
            } else if (m_IntakeCurrentState != IntakeStates.StateStartup) {
                m_talon_winchR.setControl(posWinchRequestR.withPosition(0));
                m_talon_winchL.setControl(posWinchRequestL.withPosition(0));

            }
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
