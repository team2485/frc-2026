package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.AimConstants.*;



public class TargetTracking extends SubsystemBase {
    private Drivetrain m_drivetrain;
    private PoseEstimation m_PoseEstimation;
    private CommandXboxController driverController;
    private TargetingStates currentState = TargetingStates.StateDriverControlled;
    private TargetingStates requestedState = TargetingStates.StateDriverControlled;
    private FieldCentric m_manualDrive;
    private PIDController m_PidController= new PIDController(20,5,0.1); // TODO: Looks good right now but may need retuning later down the line :(
    private Pose2d aimPosition = new Pose2d();

    private double currentRotation;
    private double targetRotation;
    DoublePublisher targetPublisher;
    DoublePublisher anglePublisher;
    BooleanPublisher targetLockPublisher;
    
    // private CommandXboxController operatorController;
    public enum TargetingStates {
        StateIdle,
        StateDriverControlled,
        StateAiming,
        StateDriveToAimTransition,
        StateResetHeading,
    }
     

    public TargetTracking(Drivetrain dt, PoseEstimation pe, CommandXboxController cxc, FieldCentric manualDrive) {
        m_drivetrain = dt;
        m_PoseEstimation = pe;
        driverController = cxc;
        m_manualDrive = manualDrive;
        m_PidController.setTolerance(0.001);
        // m_PidController = pid;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // Get the table within that instance that contains the data. There can
        // be as many tables as you like and exist to make it easier to organize
        // your data. In this case, it's a table called Target tracking.
        NetworkTable table = inst.getTable("Target Tracking");

        targetPublisher = table.getDoubleTopic("targetRotation").publish(); // Various outputs the driver may want :)
        anglePublisher = table.getDoubleTopic("currentRotation").publish();
        targetLockPublisher = table.getBooleanTopic("target locked").publish();
        // targetPosePub = table.getTopic("target pose").genericPublish(getName(), null)
        m_PidController.enableContinuousInput(-0.5, 0.5); // rotations
    }

    @Override
    public void periodic() {
        
      
        switch (currentState) {
            case StateIdle:

                break;
            // case StateResetHeading: replaced by a command since it works better with phoenixtuner's stuff
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
        recalculateAimPosition();
        stateSwitchLogic();
    }

    public void recalculateAimPosition(){
        // return new Pose2d(11.864, 4.626, new Rotation2d(0)); // 

        ChassisSpeeds curVelo = ChassisSpeeds.fromRobotRelativeSpeeds(m_drivetrain.getState().Speeds, m_drivetrain.getState().Pose.getRotation()); // find a field relative velocity of the chassis
        Pose2d newAimPose = new Pose2d(getHubPose().getX() - curVelo.vxMetersPerSecond*1.5 ,getHubPose().getY() - curVelo.vyMetersPerSecond*1.5 , Rotation2d.k180deg); // subtract it from the pose of the target
        aimPosition = newAimPose;
        // aimPosition= new Pose2d( 11.91, 4.0345, new Rotation2d(0));
    }
    public Rotation2d calculateTargetLockAngle(){
        

        Translation2d selfPose = m_drivetrain.getState().Pose.getTranslation();
        Translation2d dif = aimPosition.getTranslation().minus(selfPose);
        return dif.getAngle();



    }
    public Pose2d getHubPose() {
        // return new Pose2d(VisionConstants.kBlueTagList.get(14).pose.getTranslation().toTranslation2d().plus(new Translation2d(20 * kInchesToMeters,0)), Rotation2d.kZero);\
        return new Pose2d(11.91, 4.0345, new Rotation2d(0)); // 
        // 4.626 from baseline
        // 4.035 from sideline
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
    public Pose2d getAimPose(){

        return aimPosition;

    }
    public Command alignToHub(Drivetrain m_drivetrain, PoseEstimation m_poseEstimation) {

        Pose2d targetPose = getHubPose(); /// deprecating this and putting it into different member functions that more properly implement wpilib's kinematics classes :p
        // SmartDashboard.putData("hi",targetPose);
        double distX = targetPose.getTranslation().getX() - m_poseEstimation.getCurrentPose().getX();
        double distY = targetPose.getTranslation().getY() - m_poseEstimation.getCurrentPose().getY();

        
        // double dist = m_poseEstimation.getCurrentPose().getTranslation().getDistance();

        Rotation2d hub = new Rotation2d(Math.PI/3);
        Rotation2d currentRotation = m_drivetrain.getState().Pose.getRotation();

        Rotation2d error = currentRotation.relativeTo(hub);
        
        double targetAngleRadians = hub.getRadians();
        double currentAngleRotations = currentRotation.getRotations();
        final double currentAngleRadiansFinal = (currentAngleRotations - (int) currentAngleRotations) * 2 * Math.PI; // plugging in different math stuff
        final double targetAngleRadiansFinal = calculateTargetLockAngle().getRadians() ;
 
        // while(targetAngleRadians < 0)
        // {
            
        //    targetAngleRadians += Math.PI *2;
        // } 

        //  while(currentAngleRadians < 0)
        // {
        //    currentAngleRadians += Math.PI *2;
        // }
        // final double targetAngleRadiansFinal = targetAngleRadians;
       targetPublisher.set(targetAngleRadiansFinal- currentAngleRadiansFinal);
        anglePublisher.set(currentAngleRadiansFinal);
        if(Math.abs(currentAngleRadiansFinal - targetAngleRadiansFinal) < AimConstants.kTargetAngleTolerance){
            targetLockPublisher.set( true);
        }
        else{
            targetLockPublisher.set( false);
        }
        return m_drivetrain.applyRequest( // Allows for some SWIM control but only .5x speed on the drivetrain.
                                () -> m_manualDrive.withVelocityX(-driverController.getLeftY() * Constants.MaxSpeed * .2) // Drive
                                                                                                                     // forward
                                                                                                                     // with
                                                                                                                     // negative
                                                                                                                     // Y
                                                                                                                     // (forward)
                                        .withVelocityY(-driverController.getLeftX() * Constants.MaxSpeed * .2)
                                                                                                          // with
                                                                                                          // negative X
                                                                                                          // (left)
                                        .withRotationalRate( m_PidController.calculate((currentAngleRadiansFinal/(2*Math.PI)), (targetAngleRadiansFinal/(2*Math.PI)))) // Drive // +5*Math.PI/4
                                                                                                                      // counterclockwise
                                                                                                                      // with
                                                                                                                      // negative
                                                                                                                      // X
                                                                                                                      // (left)
                        ); 

    }
}
