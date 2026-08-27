package frc.robot.subsystems.drive;

import static frc.robot.Constants.Swerve.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.*;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        // /* Swerve Angle Motor Configurations */
        // SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     angleEnableCurrentLimit,
        //     angleContinuousCurrentLimit,
        //     anglePeakCurrentLimit,
        //     anglePeakCurrentDuration);

        swerveAngleFXConfig.Slot0.kP = angleKP;
        swerveAngleFXConfig.Slot0.kI = angleKI;
        swerveAngleFXConfig.Slot0.kD = angleKD;
        swerveAngleFXConfig.Slot0.kS = 0;
        swerveAngleFXConfig.Slot0.kV = angleKV;
        swerveAngleFXConfig.Slot0.kA = 0;


        // swerveAngleFXConfig.Slot0.kF = angleKF;
        swerveAngleFXConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = anglePeakCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerLimit = angleContinuousCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerTime = anglePeakCurrentDuration;

        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = angleGearRatio;


        /* Swerve Drive Motor Configuration */
        // SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     driveEnableCurrentLimit,
        //     driveContinuousCurrentLimit,
        //     drivePeakCurrentLimit,
        //     drivePeakCurrentDuration);

        swerveDriveFXConfig.Slot0.kP = driveKP;
        swerveDriveFXConfig.Slot0.kI = driveKI;
        swerveDriveFXConfig.Slot0.kD = driveKD;
        // swerveDriveFXConfig.Slot0.kF = driveKF;
        // swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        // swerveDriveFXConfig.openloopRamp = openLoopRamp;
        // swerveDriveFXConfig.closedloopRamp = closedLoopRamp;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = drivePeakCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = driveContinuousCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = drivePeakCurrentDuration;

        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = driveGearRatio;

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =0.5 ; // TODO: what does this mean? Used to be AbsoluteSensorRange, removed in 2025?
        swerveCanCoderConfig.MagnetSensor.MagnetOffset = .5;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = canCoderInvert ? SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;
        // swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        // swerveCanCoderConfig.sensorDirection = canCoderInvert;
        // swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
