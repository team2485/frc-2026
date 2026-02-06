package frc.robot.subsystems.drive;

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

public class TargetTracking extends SubsystemBase {
    private Drivetrain m_drivetrain;
    private PoseEstimation m_PoseEstimation;
    private CommandXboxController driverController;
    private TargetingStates currentState = TargetingStates.StateDriverControlled;
    private TargetingStates requestedState = TargetingStates.StateDriverControlled;
    private FieldCentric m_manualDrive;
    private PIDController m_PidController;
    private double currentRotation;
    private double targetRotation;
     DoublePublisher xPub;
      DoublePublisher yPub;

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

         NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // Get the table within that instance that contains the data. There can
    // be as many tables as you like and exist to make it easier to organize
    // your data. In this case, it's a table called datatable.
    NetworkTable table = inst.getTable("vars");
    // Start publishing topics within that table that correspond to the X and Y values
    // for some operation in your program.
    // The topic names are actually "/datatable/x" and "/datatable/y".
    xPub = table.getDoubleTopic("targetRotation").publish();
    yPub = table.getDoubleTopic("currentRotation").publish();

    }

    @Override
    public void periodic() {
        
      
        switch (currentState) {
            case StateIdle:

                break;
            // case StateResetHeading:
            //     if (m_drivetrain.getCurrentCommand() != null) {

            //         m_drivetrain.getCurrentCommand().cancel();
            //     }   
            
            //     // m_drivetrain.runOnce(m_drivetrain::seedFieldCentric);
            //     // m_drivetrain.seedFieldCentric();
            //     m_drivetrain.resetGyro()
            //     // m_drivetrain.setOperatorPerspectiveForward(m_drivetrain.getPigeon2().getRotation2d().unaryMinus());
            //     // m_drivetrain.resetRotation();
            //     requestedState = TargetingStates.StateDriverControlled;
            //     // currentState = TargetingStates.StateDriverControlled;
            //     break;
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

        Rotation2d hub = new Rotation2d(Math.PI/4);
        Rotation2d currentRotation = m_poseEstimation.getCurrentPose().getRotation();

        Rotation2d error = currentRotation.relativeTo(hub);
        
        double targetAngleRadians = hub.getRadians();
        double currentAngleRadians = currentRotation.getRadians();

        while(targetAngleRadians < 0)
        {
            
           targetAngleRadians += Math.PI *2;
        } 

         while(currentAngleRadians < 0)
        {
           currentAngleRadians += Math.PI *2;
        }
        final double targetAngleRadiansFinal = targetAngleRadians;
        final double currentAngleRadiansFinal = currentAngleRadians;
        
        xPub.set(targetAngleRadiansFinal);
        yPub.set(currentAngleRadiansFinal);

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
                                        .withRotationalRate(m_PidController.calculate((currentAngleRadiansFinal), (targetAngleRadiansFinal))) // Drive // +5*Math.PI/4
                                                                                                                      // counterclockwise
                                                                                                                      // with
                                                                                                                      // negative
                                                                                                                      // X
                                                                                                                      // (left)
                        ); // * Constants.MaxAngularRate

    }
}
