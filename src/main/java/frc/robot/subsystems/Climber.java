package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

@Deprecated
public class Climber extends SubsystemBase {

    public enum ClimberStates {
        StateIdle,
        StateExtend,
        StateClimb
    }

    public ClimberStates m_climberCurrentState;
    public ClimberStates m_climberRequestedState;

    private final TalonFX m_talon = new TalonFX(kClimberPort, "Other");
    private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
    private double desiredPosition;
    private double desiredVoltage;

    public Climber() {
        desiredPosition = 0;
        desiredVoltage = 0;

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
        talonFXConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 60;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 100;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
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
        switch (m_climberCurrentState) {
            case StateIdle:
                desiredVoltage = 0;
                m_talon.setControl(new NeutralOut());
                break;
            case StateExtend:
                // desiredPosition = 2;
                desiredVoltage = 3;
                break;
            case StateClimb:
                // desiredPosition = 1;
                desiredVoltage = -3;
                break;
        }
        desiredPosition *= 23; // gear ratio
        runControlLoop();

        // if (getError() < kClimberErrorTolerance)
        //     m_climberCurrentState = m_climberRequestedState;
        // else
        //     m_climberCurrentState = ClimberStates.StateMovingUp;
    }

    public void runControlLoop() {
        // m_talon.setControl(request.withPosition(desiredPosition));
        m_talon.setVoltage(desiredVoltage);
        m_climberCurrentState = m_climberRequestedState;
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
/*
Make climber positional and
Zero at pulled in position
Let out slack
Upon pressing button, go back one rotation
*/
