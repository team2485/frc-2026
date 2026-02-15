package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class Climber extends SubsystemBase {

    public enum ClimberStates {
        StateIdle,
        StateMovingUp,
        StateMovingDown,
        StateUp
    }

    public ClimberStates m_climberCurrentState;
    public ClimberStates m_climberRequestedState;

    private final TalonFX m_talon = new TalonFX(kClimberPort);
    private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
    private double desiredPosition;

    public Climber() {
        desiredPosition = 0;

        var talonFXConfigs = new TalonFXConfiguration();
        // These will be derived experimentally but in case you are wondering
        // How these terms are defined from the TalonFX docs
        // kS adds n volts to overcome static friction
        // kV outputs n volts when the velocity target is 1 rotation per second
        // kP outputs 12 volts when the positional error is 12/n rotations
        // kI adds n volts per second when the positional error is 1 rotation
        // kD outputs n volts when the velocity error is 1 rotation per second
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = kSClimber;
        slot0Configs.kV = kVClimber;
        slot0Configs.kP = kPClimber;
        slot0Configs.kI = kIClimber;
        slot0Configs.kD = kDClimber;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = kClimberVelocity;
        // vel/acc = time to reach constant velocity
        motionMagicConfigs.MotionMagicAcceleration = kClimberAcceleration;
        // acc/jerk = time to reach constant acceleration
        // motionMagicConfigs.MotionMagicJerk = kClimberJerk;

        var motorOutputConfigs = talonFXConfigs.MotorOutput;
        if (kClimberClockwisePositive)
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        else
            motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        m_talon.getConfigurator().apply(talonFXConfigs);

        m_climberCurrentState = ClimberStates.StateIdle;
        m_climberRequestedState = ClimberStates.StateIdle;

        m_talon.setPosition(0);
    }

    @Override
    public void periodic() {
        switch (m_climberRequestedState) {
            case StateIdle:
                m_talon.stopMotor();
            case StateMovingUp:
                desiredPosition = 18;
            case StateMovingDown:
                desiredPosition = -18;
            case StateUp:
                m_talon.stopMotor();
        }
        runControlLoop();

        if (getError() < kClimberErrorTolerance)
            m_climberCurrentState = m_climberRequestedState;
        else
            m_climberCurrentState = ClimberStates.StateMovingUp;
    }

    public void runControlLoop() {
        m_talon.setControl(request.withPosition(desiredPosition));
    }

    private double getPosition() {
        return m_talon.getPosition().getValueAsDouble();
    }

    public double getError() {
        return Math.abs(getPosition() - desiredPosition);
    }

    // example of a "setter" method
    public void requestState(ClimberStates requestedState) {
        m_climberRequestedState = requestedState;
    }

    // example of a "getter" method
    public ClimberStates getCurrentState() {
        return m_climberCurrentState;
    }

}
