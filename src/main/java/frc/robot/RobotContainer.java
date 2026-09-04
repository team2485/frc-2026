// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder.IndexingType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.subsystems.Angler.AnglerStates;
import frc.robot.subsystems.Feeder.FeederStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.Windexer.WindexerStates;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.PoseEstimation;
import frc.robot.subsystems.drive.TargetTracking;
import frc.robot.subsystems.drive.TargetTracking.TargetingStates;

public class RobotContainer {

    private final CommandXboxController m_driver = new CommandXboxController(0);
    private final CommandXboxController m_operator = new CommandXboxController(1);
    public final Drivetrain m_drivetrain = new Drivetrain();
    public final PoseEstimation m_poseEstimation = new PoseEstimation(m_drivetrain::getYawForOdometry,
            m_drivetrain::getModulePositions, m_drivetrain::getChassisSpeeds, m_drivetrain);
    public final Telemetry logger = new Telemetry(Constants.Swerve.maxSpeed, m_drivetrain, m_poseEstimation);
//     public final PIDController m_PidController = 
    public final Shooter m_shooter = new Shooter(this);
    public final Feeder m_feeder = new Feeder(this);
    public final Intake m_intake = new Intake();

    public final TargetTracking tracker = new TargetTracking(this, m_drivetrain, m_poseEstimation, m_driver, m_operator);
    public final Angler m_angler = new Angler(m_drivetrain,tracker,this,m_shooter);
    public final Windexer m_windexer = new Windexer(this);
    public final AutoStateMachine autoStateMachine = new AutoStateMachine(this);
    // public final AutoStateMachine autoController = new AutoStateMachine(this);

    // Draggables (Visualizing mechanisms in 3D using AdvantageScope)
    @AutoLogOutput(key = "Draggables/SwerveMeasuredStates")
    public static SwerveModuleState[] swerveMeasuredStates = new SwerveModuleState[] {};
    
    @AutoLogOutput(key = "Draggables/SwerveDesiredStates")
    public static SwerveModuleState[] swerveDesiredStates = new SwerveModuleState[] {};

    @AutoLogOutput(key = "Draggables/Components3d")
    public static Pose3d[] components3d = new Pose3d[] { new Pose3d(), new Pose3d(), new Pose3d() };

    @AutoLogOutput(key = "Draggables/DesiredComponents3d")
    public static Pose3d[] desiredComponents3d = new Pose3d[] { new Pose3d(), new Pose3d(), new Pose3d() };

    @AutoLogOutput(key = "Draggables/RobotPose2d")
    public static Pose2d robotPose2d = Pose2d.kZero;

    public RobotContainer() {
        configureBindings();
        PathfindingCommand.warmupCommand().schedule();
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // m_drivetrain.setDefaultCommand(
        // // Drivetrain will execute this command periodically
        // m_drivetrain.applyRequest(() ->
        // drive.withVelocityX(-m_driver.getLeftY() * MaxSpeed) // Drive forward with
        // negative Y (forward)
        // .withVelocityY(-m_driver.getLeftX() * MaxSpeed) // Drive left with negative X
        // (left)
        // .withRotationalRate(-m_driver.getRightX() * MaxAngularRate) // Drive
        // counterclockwise with negative X (left)
        // )
        // );

        m_driver.a().whileTrue(m_drivetrain.run(
                () -> m_drivetrain.drive(new Translation2d(), 0, false, true, new Translation2d())));
        // m_driver.b().whileTrue(m_drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-m_driver.getLeftY(),
        // -m_driver.getLeftX()))
        // ));
        // m_driver.y().onTrue(() ->
        // tracker.requestState(TargetingStates.StateDriverControlled));

        m_driver.rightTrigger().onFalse(new InstantCommand(() -> tracker.requestState(TargetingStates.StateDriverControlled)));
        m_driver.rightTrigger().onTrue(new InstantCommand(() -> tracker.requestState(TargetingStates.StateDriveToAimTransition)));
        m_driver.povDown().onTrue((new InstantCommand(() -> m_intake.requestState(IntakeStates.StateShooting))));

        // m_driver.x().onTrue(m_drivetrain.resetGyro());
        m_driver.x().onTrue(new InstantCommand(() -> tracker.requestState(TargetingStates.StateResetHeading)));
        // Shooting is now bound to one button
        m_operator.rightTrigger().onTrue(new InstantCommand(() -> m_shooter.requestState(ShooterStates.StateAccelerating))); // Flywheels (shooter)
        m_operator.rightTrigger().onFalse(new InstantCommand(() -> m_shooter.requestState(ShooterStates.StateDeccelerating)).andThen(new InstantCommand(() -> m_feeder.requestState(FeederStates.StateOff))));
        // m_operator.rightTrigger().onTrue(new InstantCommand(() -> m_feeder.requestState(FeederStates.StateFeeding)));
        // m_operator.rightTrigger().onFalse(new InstantCommand(() -> m_feeder.requestState(FeederStates.StateOff)));
        m_operator.leftTrigger().onTrue(new InstantCommand(() -> m_shooter.requestState(ShooterStates.StateLongPass)).andThen(new WaitCommand(1)).andThen(new InstantCommand(() -> m_feeder.requestState(FeederStates.StateFORCE))).andThen(new InstantCommand(() -> m_windexer.requestState(WindexerStates.StateFORCE))));
        m_operator.leftTrigger().onFalse(new InstantCommand(() -> m_shooter.requestState(ShooterStates.StateDeccelerating)).andThen(new InstantCommand(() -> m_feeder.requestState(FeederStates.StateOff))).andThen(new InstantCommand(() -> m_windexer.requestState(WindexerStates.StateZero))));
         
        m_operator.rightTrigger().onFalse(new InstantCommand(() -> m_shooter.requestState(ShooterStates.StateDeccelerating)).andThen(new InstantCommand(() -> m_feeder.requestState(FeederStates.StateOff))));
        // m_operator.rightTrigger().onTrue(new InstantCommand(() -> m_feeder.requestState(FeederStates.StateFeeding)));
        // m_operator.right().onTrue(new InstantCommand(() -> m_shooter.requestState(ShooterStates.StateLongPass)).andThen(new InstantCommand(() -> m_feeder.requestState(FeederStates.StateFeeding))).andThen(new InstantCommand(() -> m_windexer.requestState(WindexerStates.StateFORCE))));

        m_operator.povUp().onTrue(new InstantCommand(() -> m_angler.requestState(AnglerStates.StateMax)));
        m_operator.povUp().onFalse(new InstantCommand(() -> m_angler.requestState(AnglerStates.StateZero)));
        m_operator.povDown().onTrue(new InstantCommand(() -> m_angler.requestState(AnglerStates.StateZero)));
        // m_operator.povLeft().onTrue(new InstantCommand(() -> m_angler.requestState(AnglerStates.State)));
        // m_operator.povDown().onTrue(new InstantCommand(() -> m_angler.requestState(AnglerStates.StateZero)));

        // m_operator.povLeft().onTrue(new InstantCommand(() -> m_angler.requestState(AnglerStates.StateTest1)));
        // m_operator.povRight().onTrue(new InstantCommand(() -> m_angler.requestState(AnglerStates.StateTest2)));
        // m_operator.b().onTrue(new InstantCommand(() -> m_angler.requestState(AnglerStates.StateTest4)));

        m_operator.a().onTrue(new InstantCommand((() -> m_angler.requestState(AnglerStates.StateAuto))));
        m_operator.a().onFalse(new InstantCommand((() -> m_angler.requestState(AnglerStates.StateZero))));

        m_operator.y().onTrue(new InstantCommand(() -> m_angler.requestState(AnglerStates.StateAmplify)));


        m_driver.leftTrigger().onTrue(new InstantCommand(() -> m_intake.requestState(IntakeStates.StateIntaking)));
        m_driver.leftTrigger().onFalse(new InstantCommand(() -> m_intake.requestState(IntakeStates.StateExtendedIdlingWheels)));
        // m_driver.leftTrigger().onFalse(new InstantCommand(() -> m_intake.requestState(IntakeStates.StateIdle)));
        m_driver.leftBumper().onTrue(new InstantCommand(() -> m_intake.requestState(IntakeStates.StateRetracting)));
        m_driver.leftBumper().onFalse(new InstantCommand(() -> m_intake.requestState(IntakeStates.StateRetracted)));
        m_driver.rightBumper().onTrue(new InstantCommand(() -> m_intake.requestState(IntakeStates.StateSlowRetract)));
        m_driver.b().onTrue(new InstantCommand(() -> m_intake.requestState(IntakeStates.StateOuttaking)));

        m_operator.leftBumper().onTrue(new InstantCommand(() -> m_windexer.requestState(WindexerStates.StateFeed)).andThen(new InstantCommand(() -> m_feeder.requestState(FeederStates.StateFeeding))));
        m_operator.leftBumper().onFalse(new InstantCommand(() -> m_windexer.requestState(WindexerStates.StateZero)).andThen(new InstantCommand(() -> m_feeder.requestState(FeederStates.StateOff))));

        //m_operator.leftBumper().onTrue(new InstantCommand(() -> m_windexer.requestState(WindexerStates.StateReverse)));
        m_operator.x().onTrue(new InstantCommand(() -> m_windexer.requestState(WindexerStates.StateZero))); // These overide the auto windexer control

  
        // Reset the field-centric heading on X press.
        // m_driver.x().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                new InstantCommand(m_drivetrain::zeroGyro)
                        .alongWith(new InstantCommand(m_drivetrain::resetToAbsolute)),
                // Then slowly drive forward (away from us) for 5 seconds.
                m_drivetrain.run(
                        () -> m_drivetrain.drive(new Translation2d(0.5, 0), 0, false, true, new Translation2d()))
                        .withTimeout(5.0),
                // Finally stop for the rest of auton
                m_drivetrain.run(
                        () -> m_drivetrain.drive(new Translation2d(), 0, false, true, new Translation2d())));
    }
}
