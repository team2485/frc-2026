package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class TargetTracking extends SubsystemBase {
    private Drivetrain m_drivetrain;
    private PoseEstimation m_PoseEstimation;
    private CommandXboxController driverController;
    private TargetingStates currentState = TargetingStates.StateDriverControlled;
    private TargetingStates requestedState = TargetingStates.StateDriverControlled;
    private FieldCentric m_manualDrive;
    private PIDController m_PidController;

    // private CommandXboxController operatorController;
    public enum TargetingStates {
        StateIdle,
        StateDriverControlled,
        StateAiming,
        StateDriveToAimTransition,
        StateResetHeading,
    }

    public TargetTracking(Drivetrain dt, PoseEstimation pe, CommandXboxController cxc, FieldCentric manualDrive, PIDController pid) {
        m_drivetrain = dt;
        m_PoseEstimation = pe;
        driverController = cxc;
        m_manualDrive = manualDrive;
        m_PidController = pid;

    }

    @Override
    public void periodic() {

        switch (currentState) {
            case StateIdle:

                break;
            case StateResetHeading:
                if (m_drivetrain.getCurrentCommand() != null) {

                    m_drivetrain.getCurrentCommand().cancel();
                }   
            
                // m_drivetrain.runOnce(m_drivetrain::seedFieldCentric);
                // m_drivetrain.seedFieldCentric();
                
                m_drivetrain.setOperatorPerspectiveForward(m_drivetrain.getPigeon2().getRotation2d());
                // m_drivetrain.resetRotation();
                requestedState = TargetingStates.StateDriverControlled;
                // currentState = TargetingStates.StateDriverControlled;
                break;
            case StateDriverControlled:
            //     driverController.x().whileTrue(new InstantCommand( () -> 
            //    // joystick.x().onTrue()
            //         m_drivetrain.runOnce(m_drivetrain::seedFieldCentric)
                
            //     ));
                // m_drivetrain.setOperatorPerspectiveForward(m_drivetrain.getPigeon2().getRotation2d());
                
                CommandScheduler.getInstance()
                        .schedule(m_drivetrain.applyRequest(
                                () -> m_manualDrive.withVelocityX(-driverController.getLeftY() * Constants.MaxSpeed) // Drive
                                                                                                                     // forward
                                                                                                                     // with
                                                                                                                     // negative
                                                                                                                     // Y
                                                                                                                     // (forward)
                                        .withVelocityY(-driverController.getLeftX() * Constants.MaxSpeed) // Drive left
                                                                                                          // with
                                                                                                          // negative X
                                                                                                          // (left)
                                        .withRotationalRate(-driverController.getRightX() * Constants.MaxAngularRate) // Drive
                                                                                                                      // counterclockwise
                                                                                                                      // with
                                                                                                                      // negative
                                                                                                                      // X
                                                                                                                      // (left)
                        ));
                break;
            case StateDriveToAimTransition:
                if (m_drivetrain.getCurrentCommand() != null) {

                    m_drivetrain.getCurrentCommand().cancel();
                }
                CommandScheduler.getInstance()
                        .schedule(m_drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
                requestedState = TargetingStates.StateAiming;
                break;
            case StateAiming:
                CommandScheduler.getInstance().schedule(alignToHub(m_drivetrain, m_PoseEstimation));
                break;
            default:
                break;
        }
        stateSwitchLogic();
    }

    public void stateSwitchLogic() {

        currentState = requestedState; // Put the checks for switching states here :)

    }

    public void requestState(TargetingStates req) {
        requestedState = req;
    }

    public TargetingStates getState() {
        return currentState;

    }

    public TargetingStates getRequestedState() {
        return requestedState;

    }

    public Command alignToHub(Drivetrain m_drivetrain, PoseEstimation m_poseEstimation) {

        // Since we are using a holonomic drivetrain, the rotation component of this
        // pose
        // represents the goal holonomic rotation
        Pose2d targetPose = m_poseEstimation.getHubPose();

        double distX = targetPose.getTranslation().getX() - m_poseEstimation.getCurrentPose().getX();
        double distY = targetPose.getTranslation().getY() - m_poseEstimation.getCurrentPose().getY();
        // double dist = m_poseEstimation.getCurrentPose().getTranslation().getDistance();

        Rotation2d hub = new Rotation2d(distX, distY);
        Rotation2d currentRotation = m_drivetrain.getRotation3d().toRotation2d();

        Rotation2d error = currentRotation.relativeTo(hub);

        return m_drivetrain.applyRequest(
                                () -> m_manualDrive.withVelocityX(0) // Drive
                                                                                                                     // forward
                                                                                                                     // with
                                                                                                                     // negative
                                                                                                                     // Y
                                                                                                                     // (forward)
                                        .withVelocityY(0) // Drive left
                                                                                                          // with
                                                                                                          // negative X
                                                                                                          // (left)
                                        .withRotationalRate(m_PidController.calculate(currentRotation.getDegrees(), hub.getDegrees())) // Drive
                                                                                                                      // counterclockwise
                                                                                                                      // with
                                                                                                                      // negative
                                                                                                                      // X
                                                                                                                      // (left)
                        ); // * Constants.MaxAngularRate

    }
}
