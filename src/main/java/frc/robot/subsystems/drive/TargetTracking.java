package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.AimConstants.*;
import frc.robot.subsystems.Spindexer.SpindexerStates;
import frc.util.DistanceLookup;




public class TargetTracking extends SubsystemBase {
    private Drivetrain m_drivetrain;
    private PoseEstimation m_PoseEstimation;
    private CommandXboxController driverController;
    private TargetingStates currentState = TargetingStates.StateDriverControlled;
    private TargetingStates requestedState = TargetingStates.StateDriverControlled;
    private FieldCentric m_manualDrive; // 24 7 2
    // new PIDController(25,7,1.75);
    private PIDController m_PidController= new PIDController(25,5,2); // TODO: Looks good right now but may need retuning later down the line :(
    private Pose2d aimPosition = new Pose2d();
   StructPublisher aimPosePub = NetworkTableInstance.getDefault().getStructTopic("AimPose", Pose2d.struct).publish();
    private double currentRotation;
    private double targetRotation;
    DoublePublisher targetPublisher;
    DoublePublisher anglePublisher;
    BooleanPublisher targetLockPublisher;
    CommandXboxController opController;
    public boolean targetLocked=false;
    private RobotContainer m_robotContainer;
    // private CommandXboxController operatorController;
    public enum TargetingStates {
        StateIdle,
        StateDriverControlled,
        StateAiming,
        StateDriveToAimTransition,
        StateResetHeading,
    }
     

    public TargetTracking(RobotContainer bot, Drivetrain dt, PoseEstimation pe, CommandXboxController cxc,CommandXboxController cxc2,FieldCentric manualDrive) {
        m_drivetrain = dt;
        m_robotContainer = bot;
        m_PoseEstimation = pe;
        driverController = cxc;
        opController = cxc2;
        m_manualDrive = manualDrive;
        m_PidController.setTolerance(0.001);
        // m_PidController = pid;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // Get the table within that instance that contains the data. There can
        // be as many tables as you like and exist to make it easier to organize
        // your data. In this case, it's a table called Target tracking.
        NetworkTable table = inst.getTable("Target Tracking");

        targetPublisher = table.getDoubleTopic("Aim Error").publish(); // Various outputs the driver may want :)
        anglePublisher = table.getDoubleTopic("Current Rotation").publish();
        targetLockPublisher = table.getBooleanTopic("Target Locked").publish();
        // targetPosePub = table.getTopic("target pose").genericPublish(getName(), null)
        m_PidController.enableContinuousInput(-0.5, 0.5); // rotations
    }

    // @Override
    public void periodic() {
        stateSwitchLogic();

        switch (currentState) {
            case StateIdle:
                    aimPosePub.set(Pose2d.kZero);

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
                    driverController.setRumble(RumbleType.kBothRumble, 0);
                    opController.setRumble(RumbleType.kBothRumble, 0);

                    aimPosePub.set(Pose2d.kZero);
                    if(!DriverStation.isTeleopEnabled()){
                        break;
                    }
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
                    aimPosePub.set(aimPosition);

                if (m_drivetrain.getCurrentCommand() != null) {

                    m_drivetrain.getCurrentCommand().cancel();
                }
                CommandScheduler.getInstance()
                        .schedule(m_drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
                requestedState = TargetingStates.StateAiming;
                break;
            case StateAiming:
                aimPosePub.set(aimPosition);

                CommandScheduler.getInstance().schedule(alignToHub(m_drivetrain, m_PoseEstimation));
                // var targetLocked = ;
                if(targetLocked){

                    m_robotContainer.m_spindexer.requestState(SpindexerStates.StateAutomatedEnable);
                    driverController.setRumble(RumbleType.kBothRumble, 0);
                    opController.setRumble(RumbleType.kBothRumble, 0);
                    

                }else{

                    m_robotContainer.m_spindexer.requestState(SpindexerStates.StateAutomatedOff);
                    driverController.setRumble(RumbleType.kBothRumble, 1);
                    opController.setRumble(RumbleType.kBothRumble, 1);

                }
                
                break;
            default:
                break;
        }
        
        recalculateAimPosition();
        
    }

    public void recalculateAimPosition(){
        double t = DistanceLookup.getTime(m_drivetrain.getState().Pose.getTranslation()
                .getDistance(aimPosition.getTranslation())); // find time variable from lookup table
        ChassisSpeeds curVelo = ChassisSpeeds.fromRobotRelativeSpeeds(
                m_drivetrain.getState().Speeds, 
                m_drivetrain.getState().Pose.getRotation()); // find a field relative velocity of the chassis
        // Pose2d newAimPose = new Pose2d(getHubPose().getX() - curVelo.vxMetersPerSecond*t , // factor in time to hit
        //         getHubPose().getY() - curVelo.vyMetersPerSecond*t ,
        //          Rotation2d.k180deg); // subtract it from the pose of the target to create opposing aim angle
        aimPosition = getHubPose();
    }
    public Rotation2d calculateTargetLockAngle(){
        Translation2d selfPose = m_drivetrain.getState().Pose.getTranslation();
        Translation2d dif = aimPosition.getTranslation().minus(selfPose);
        return dif.getAngle();
    }
    public Pose2d getHubPose() {
        // Pose2d bias = new Pose2d(opController.getLeftX(), opController.getLeftY(),Rotation2d.kZero);
        
        
        if(DriverStation.getAlliance().get() == Alliance.Red){

            return new Pose2d(11.91, 4.0345, new Rotation2d(0)).plus(new Transform2d(-1*opController.getLeftX(), opController.getLeftY(), Rotation2d.kZero)); // Red hub from blue origin TODO add team switching logics


        }
        else{
            return new Pose2d(4.6228, 4.0345, new Rotation2d(0)).plus(new Transform2d(opController.getLeftX(), opController.getLeftY(), Rotation2d.kZero)); // Red hub from blue origin TODO add team switching logics

        }
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
        double distY = targetPose.getTranslation().getY() - m_poseEstimation.getCurrentPose().getY(); // These values are deprecated and should be ignored

        
        // double dist = m_poseEstimation.getCurrentPose().getTranslation().getDistance();

        // Rotation2d hub = new Rotation2d(Math.PI/3);
        Rotation2d currentRotation = m_drivetrain.getState().Pose.getRotation();

        // Rotation2d error = currentRotation.relativeTo(hub);
        
        // double targetAngleRadians = hub.getRadians();
        double currentAngleRotations = currentRotation.getRotations();
        final double currentAngleRadiansFinal = (currentAngleRotations - (int) currentAngleRotations) * 2 * Math.PI; // plugging in different math stuff
        final double targetAngleRadiansFinal = calculateTargetLockAngle().getRadians() ;
 
        // while(targetAngleRadians < 0) Shouldn't be needed since I put the calculations into rotations and bounded the PID -.5 to .5
        // {
            
        //    targetAngleRadians += Math.PI *2;
        // } 

        //  while(currentAngleRadians < 0)
        // {
        //    currentAngleRadians += Math.PI *2;
        // }
        // final double targetAngleRadiansFinal = targetAngleRadians;
        targetPublisher.set(targetAngleRadiansFinal- currentAngleRadiansFinal); // Logs the error
        anglePublisher.set(currentAngleRadiansFinal);
        var dist = m_drivetrain.getState().Pose.getTranslation()
        .getDistance(aimPosition.getTranslation());
        double tolernace = AimConstants.kTargetAngleTolerance / (dist/2.5);
        if(Math.abs(currentAngleRadiansFinal - targetAngleRadiansFinal) < tolernace){
            targetLockPublisher.set( true);
            targetLocked=true;
        }
        else{
            targetLockPublisher.set( false);
            targetLocked= false;
        }
        double aimingRateLimiter;
        double stopTurning = 1;
        if(targetLocked){
            // stopTurning=0;
            aimingRateLimiter=0;
        }
        else{
            // stopTurning=1;
            aimingRateLimiter=.2;
        }
        return m_drivetrain.applyRequest( // Allows for some SWIM control but only .5x speed on the drivetrain.
                                () -> m_manualDrive.withVelocityX(-driverController.getLeftY() * Constants.MaxSpeed * aimingRateLimiter) // Drive
                                                                                                                     // forward
                                                                                                                     // with
                                                                                                                     // negative
                                                                                                                     // Y
                                                                                                                     // (forward)
                                        .withVelocityY(-driverController.getLeftX() * Constants.MaxSpeed * aimingRateLimiter)
                                                                                                          // with
                                                                                                          // negative X
                                                                                                          // (left)
                                        .withRotationalRate( stopTurning*m_PidController.calculate((currentAngleRadiansFinal/(2*Math.PI)), (targetAngleRadiansFinal/(2*Math.PI)))) 
                                                                                          
                        ); 

    }
}
