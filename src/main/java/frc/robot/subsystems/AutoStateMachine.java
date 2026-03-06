package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.nio.file.Path;
import java.util.TreeMap;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Angler.AnglerStates;
import frc.robot.subsystems.Feeder.FeederStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.drive.TargetTracking.TargetingStates;

public class AutoStateMachine extends SubsystemBase {
    public enum AutoStates {
    StateInit,
    StateFollowingPath,
    StateIdleToFollowingPath,
    IdleToShooting,
    FollowingToShooting,
    StateShooting,
    StateWait
    }

  public PathPlannerPath PathToFollow;
  public AutoStates m_currentState=AutoStates.StateInit;
  public AutoStates m_requestedState=AutoStates.StateInit;
  public Command FollowPathCommand;

  public RobotContainer m_RobotContainer;
  public int PathNumber = 0;
  TreeMap<String, PathPlannerPath[]> autoMap = new TreeMap<>();
  public SendableChooser<String> m_Chooser = new SendableChooser<>();
 
  String chooser_val;

  public AutoStateMachine (RobotContainer m_rc){
    Shuffleboard.getTab("Autos").add(m_Chooser);
    m_Chooser.addOption("Bottom Side Shooting", "BSideShooting");
    m_Chooser.addOption("Top Side Shooting", "TSideShooting");
    m_Chooser.addOption("Test", "Straight");
    m_Chooser.setDefaultOption("BSideShoot", "BSideSimple");
    m_Chooser.addOption("Test2", "MoveAndShoot");
    m_Chooser.addOption("TSideShoot", "TopSideSimple");

    final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    try {
        
        AutoBuilder.configure(()->new Pose2d(m_rc.drivetrain.getState().Pose.getTranslation(),m_rc.drivetrain.getState().Pose.getRotation()), (pose) -> m_rc.drivetrain.fakeResetPose(pose.rotateBy(Rotation2d.fromDegrees(180))), ()->m_rc.drivetrain.getState().Speeds,
         (speeds, feedforward) -> m_rc.drivetrain.setControl(m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(new ChassisSpeeds(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond), 0.02))),//.withWheelForceFeedforwardsX(feedforward.robotRelativeForcesXNewtons()).withWheelForceFeedforwardsY(feedforward.robotRelativeForcesYNewtons())),
         new PPHolonomicDriveController(new PIDConstants(.9,0,0), new PIDConstants(1,0.0,0)),
         RobotConfig.fromGUISettings(),
         ()->DriverStation.getAlliance().orElse(Alliance.Blue)== Alliance.Red,this
         
         );
    } catch (Exception e) {
        // TODO: handle exception
        System.err.println("AUTO ABORT DUE  TO IO ERROR!");
    }

    // m_Chooser.addOption("B Auto",autoPeriodicStates.MidScoreAutoV2);  // 
    m_RobotContainer = m_rc;
    new EventTrigger("DeployIntake").onTrue(new InstantCommand( () -> {
        
        
        m_rc.m_intake.requestState(IntakeStates.StateIntaking);
        ballCountEstimate += 24;
    
    
    }) );
    new EventTrigger("Shoot").onTrue(new InstantCommand( () -> startShooting()) );
    new EventTrigger("Wait").onTrue(new InstantCommand( () -> waitTime()) );

    try {
        autoMap.put("BSideShooting", new PathPlannerPath[]{ PathPlannerPath.fromPathFile("BTrenchToMid"), 
            PathPlannerPath.fromPathFile("BMidToShooting"), 
            PathPlannerPath.fromPathFile("BShootingToDepot")
            });
        autoMap.put("MoveAndShoot", new PathPlannerPath[]{ PathPlannerPath.fromPathFile("MoveAndShoot"), 
            PathPlannerPath.fromPathFile("ShootAndMoveback")
            });
        autoMap.put("BSideSimple", new PathPlannerPath[]{ PathPlannerPath.fromPathFile("BSideTrenchShoot"), 
            PathPlannerPath.fromPathFile("BSideTrenchToDepot"),
            PathPlannerPath.fromPathFile("BSideDepotShoot")

            });
       // autoMap.get()
       autoMap.put("TSideShooting", new PathPlannerPath[]{ PathPlannerPath.fromPathFile("TTrenchToMid"), 
            PathPlannerPath.fromPathFile("TMidToShooting"), 
            PathPlannerPath.fromPathFile("TShootingToDepot"),
            PathPlannerPath.fromPathFile("TSideIntakeToShoot")

            });
        // TSideTrenchShoot TSideTrenchShootToIntake
        autoMap.put("Straight", new PathPlannerPath[] { PathPlannerPath.fromPathFile("Straight")});
        autoMap.put("TopSideSimple", new PathPlannerPath[] { PathPlannerPath.fromPathFile("TSideTrenchShoot"),
     PathPlannerPath.fromPathFile("TSideTrenchShootToIntake"),
            PathPlannerPath.fromPathFile("TSideIntakeToShoot")

    });
        
        
    } catch (Exception e) {
        // TODO: handle exception
        System.err.print("chud auto fail!");
    }

  }
  int ballCountEstimate = 8;
  boolean shootingRequested = false;
  double startShootingTime = -1;
  double waitStartTime = -1;
  @Override
  public void periodic (){
    if(!DriverStation.isAutonomousEnabled()){
        m_currentState = AutoStates.StateInit;
        m_requestedState = AutoStates.StateInit;
        return;
    }
    switch (m_currentState) {
        case StateInit:
            chooser_val = m_Chooser.getSelected();
            m_requestedState=AutoStates.StateIdleToFollowingPath;
            // m_RobotContainer.drivetrain.getPigeon2().setYaw(m_RobotContainer.m_poseEstimation.getVisionONLYPose().getRotation().getDegrees());
            // m_RobotContainer.drivetrain.resetGyro();
            break;
    
        case StateFollowingPath:
            if (FollowPathCommand.isFinished()){
                if(m_requestedState == AutoStates.IdleToShooting || m_requestedState == AutoStates.StateWait){
        
                    PathNumber++;
                    m_currentState = m_requestedState;                
                }
                else{
                    PathNumber++;

                    m_requestedState = AutoStates.StateIdleToFollowingPath;
                    m_currentState = m_requestedState;

                }
            }
            break;
        case StateWait:
            
            if(waitStartTime == -1) waitStartTime = System.currentTimeMillis();
            if(System.currentTimeMillis() - waitStartTime > 2500){


                m_requestedState = AutoStates.StateIdleToFollowingPath;
                waitStartTime = -1;
            }
            
            break;
        case StateIdleToFollowingPath:
            m_RobotContainer.tracker.requestState(TargetingStates.StateDriverControlled);
            m_RobotContainer.m_feeder.requestState(FeederStates.StateOff);
            m_RobotContainer.m_angler.requestState(AnglerStates.StateZero);
            m_RobotContainer.m_shooter.requestState(ShooterStates.StateOff);

            // CommandScheduler.getInstance()
            //             .schedule(m_RobotContainer.drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
            CommandScheduler.getInstance().cancelAll();
            if(!(autoMap.get(chooser_val).length -1 >= PathNumber)){
                break;
            }
            PathToFollow = autoMap.get(chooser_val)[PathNumber];
            if(PathToFollow == null){

                break;

            }
            if(PathNumber ==0){

            m_RobotContainer.drivetrain.resetPose(PathToFollow.flipPath().getStartingHolonomicPose().get() );


            }

            FollowPathCommand = AutoBuilder.followPath(PathToFollow);

            CommandScheduler.getInstance().schedule(FollowPathCommand);
            m_requestedState=AutoStates.StateFollowingPath;
            m_currentState=AutoStates.StateFollowingPath;
            break;
        case IdleToShooting:
            if(FollowPathCommand != null){
                FollowPathCommand.cancel();
            }
            m_RobotContainer.tracker.requestState(TargetingStates.StateDriveToAimTransition);
            m_RobotContainer.m_feeder.requestState(FeederStates.StateFeeding);
            m_RobotContainer.m_angler.requestState(AnglerStates.StateAuto);
            m_RobotContainer.m_shooter.requestState(ShooterStates.StateShooting);
            m_RobotContainer.m_intake.requestState(IntakeStates.StateRetracted);
            startShootingTime = System.currentTimeMillis();
            m_requestedState = AutoStates.StateShooting;
            break;
        case StateShooting:
            System.out.println("SHOOTING IN AUTO");
            if(System.currentTimeMillis() - startShootingTime > ((ballCountEstimate*300)+500)){ // times out the shooting

                m_requestedState = AutoStates.StateIdleToFollowingPath;
                
            }
            if(m_RobotContainer.tracker.targetLocked == false && System.currentTimeMillis() - startShootingTime > 1500){

                m_requestedState = AutoStates.StateIdleToFollowingPath;
            }
            break;
        
    }
    if(m_currentState != AutoStates.StateFollowingPath ){

        m_currentState = m_requestedState;// temp


    }
    System.out.println("Current state : " + m_currentState.toString());
    System.out.println("Req state : " + m_requestedState.toString());

  }

  public void startShooting(){
    System.out.println("wants to shoot ");
    m_requestedState = AutoStates.IdleToShooting;
   
    // m_requestedState = AutoStates.IdleToShooting;

  }
  public void waitTime(){

    m_requestedState = AutoStates.StateWait;

  }
}
