// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.PoseEstimation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveWithController extends Command {
  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_ySpeedSupplier;
  private final DoubleSupplier m_rotSpeedSupplier;
  private final BooleanSupplier m_fieldRelative;
  private final DoubleSupplier m_speedMult;
  private final Drivetrain m_drivetrain;

  private final PIDController rotationOverrideController = new PIDController(.1*.8, 0, .015*.75);
  //.175, 0, .01
  private final PIDController xOverrideController = new PIDController(5, 0, 0);
  private final PoseEstimation mPoseEstimation;
  private double speedMultiplier;

  public DriveWithController(
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier,
      DoubleSupplier rotSpeedSupplier,
      DoubleSupplier speedMultSupplier,
      BooleanSupplier fieldRelative,
      Drivetrain drivetrain,
      PoseEstimation poseEstimation) {
        this.m_speedMult = speedMultSupplier;
    this.m_xSpeedSupplier = xSpeedSupplier;
    this.m_ySpeedSupplier = ySpeedSupplier;
    this.m_rotSpeedSupplier = rotSpeedSupplier;
    this.m_fieldRelative = fieldRelative;


    this.mPoseEstimation = poseEstimation;
    this.m_drivetrain = drivetrain;

    rotationOverrideController.enableContinuousInput(-180, 180);
    addRequirements(m_drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("xbox right x", m_xSpeedSupplier.getAsDouble());
    // SmartDashboard.putNumber("xbox left x", m_ySpeedSupplier.getAsDouble());
    // SmartDashboard.putNumber("xbox left y", m_rotSpeedSupplier.getAsDouble());
    double speedMult = m_speedMult.getAsDouble();
    /* Standard WPILib frame: +X forward, +Y left, CCW+ rotation. Xbox sticks read +Y down and
     * +X right, so every axis is negated: the map(-...) below already supplies the minus for
     * translation, and rotation gets an explicit one. */
    final int xSign = (int)(Math.abs(m_xSpeedSupplier.getAsDouble())/m_xSpeedSupplier.getAsDouble());
    double xSpeed =
        map(-MathUtil.applyDeadband(Math.abs(m_xSpeedSupplier.getAsDouble()*m_xSpeedSupplier.getAsDouble()), kDriverLeftYDeadband)
            * kTeleopMaxSpeedMetersPerSecond, 0, 1, 0, Swerve.maxSpeed) * xSign;

    final int ySign = (int)(Math.abs(m_ySpeedSupplier.getAsDouble())/m_ySpeedSupplier.getAsDouble());
    double ySpeed =
        map(-MathUtil.applyDeadband(Math.abs(m_ySpeedSupplier.getAsDouble()*m_ySpeedSupplier.getAsDouble()), kDriverLeftXDeadband)
          * kTeleopMaxSpeedMetersPerSecond, 0, 1, 0, Swerve.maxSpeed) * ySign;


    final int rotSign = (int)(Math.abs(m_rotSpeedSupplier.getAsDouble())/m_rotSpeedSupplier.getAsDouble());
    double rot = Math.abs(map(-MathUtil.applyDeadband(m_rotSpeedSupplier.getAsDouble()*m_rotSpeedSupplier.getAsDouble(), kDriverRightXDeadband)
            * kTeleopMaxAngularSpeedRadiansPerSecond, 0, 1, 0, Swerve.maxAngularVelocity)) * -rotSign;

    boolean fieldRelative = m_fieldRelative.getAsBoolean();




    //Translation2d centerOfRotation = aimingAtSpeaker ? new Translation2d(-kPivotToRobot, 0) : new Translation2d();

    m_drivetrain.drive(new Translation2d(xSpeed*speedMult, ySpeed*speedMult), rot, fieldRelative, true, new Translation2d());

    // System.out.println(m_driver.getRightTriggerAxis());
  }

  private double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new Translation2d(0, 0), 0, false, false, new Translation2d());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
