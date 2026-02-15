// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.config.PIDConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants;
import frc.robot.subsystems.Angler.AnglerStates;
import frc.robot.subsystems.Feeder.FeederStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Spindexer.SpindexerStates;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.PoseEstimation;
import frc.robot.subsystems.drive.TargetTracking;
import frc.robot.subsystems.drive.TargetTracking.TargetingStates;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class RobotContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.1) // Add a 10%
                                                                                                           // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(Constants.MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController m_operator = new CommandXboxController(1);
    public final Drivetrain drivetrain = Constants.createDrivetrain();
    public final PoseEstimation m_poseEstimation = new PoseEstimation(() -> drivetrain.getPigeon2().getRotation2d(),
            () -> drivetrain.getState().ModulePositions,
            () -> drivetrain.getKinematics().toChassisSpeeds(drivetrain.getState().ModuleStates), drivetrain);
    public final PIDController m_PidController = new PIDController(1.67, 0, 0);
    public final Spindexer m_spindexer = new Spindexer();
    public final Angler m_angler = new Angler();
    public final Shooter m_shooter = new Shooter();
    public final Feeder m_feeder = new Feeder();
    public final Intake m_Intake = new Intake();

    public final TargetTracking tracker = new TargetTracking(drivetrain, m_poseEstimation, joystick, drive,
            m_PidController);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        // // Drivetrain will execute this command periodically
        // drivetrain.applyRequest(() ->
        // drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
        // negative Y (forward)
        // .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X
        // (left)
        // .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
        // counterclockwise with negative X (left)
        // )
        // );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
        // -joystick.getLeftX()))
        // ));
        // joystick.b().onTrue(new InstantCommand(() -> tracker.requestState(TargetingStates.StateDriverControlled)));
        // joystick.y().onTrue(() ->
        // tracker.requestState(TargetingStates.StateDriverControlled));
        // joystick.y().onTrue(new InstantCommand(() -> tracker.requestState(TargetingStates.StateDriveToAimTransition)));
        joystick.x().onTrue(drivetrain.resetGyro());

        joystick.rightTrigger().onTrue(new InstantCommand(() -> m_shooter.requestState(ShooterStates.StateShooting)));
        joystick.rightTrigger().onFalse(new InstantCommand(() -> m_shooter.requestState(ShooterStates.StateOff)));

        joystick.leftTrigger().onTrue(new InstantCommand(() -> m_angler.requestState(AnglerStates.StateMax)));
        // joystick.leftBumper().onTrue(new InstantCommand(() -> m_angler.requestState(AnglerStates.StateZero)));
        joystick.leftBumper().onTrue(new InstantCommand(() -> m_angler.requestState(AnglerStates.StateZero)));
        joystick.b().onTrue(new InstantCommand(() -> m_angler.requestState(AnglerStates.StateTest1)));
        joystick.y().onTrue(new InstantCommand(() -> m_angler.requestState(AnglerStates.StateTest2)));
        joystick.a().onTrue(new InstantCommand(() -> m_angler.requestState(AnglerStates.StateTest3)));

        joystick.rightBumper().onTrue(new InstantCommand(() -> m_feeder.requestState(FeederStates.StateFeeding)));
        joystick.rightBumper().onFalse(new InstantCommand(() -> m_feeder.requestState(FeederStates.StateOff)));

        m_operator.rightTrigger().onTrue(new InstantCommand(() -> m_spindexer.requestState(SpindexerStates.StateFeed)));
        m_operator.leftTrigger().onTrue(new InstantCommand(() -> m_spindexer.requestState(SpindexerStates.StateReverse)));
        m_operator.x().onTrue(new InstantCommand(() -> m_spindexer.requestState(SpindexerStates.StateZero)));

        // Reset the field-centric heading on X press.
        // joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }
}
