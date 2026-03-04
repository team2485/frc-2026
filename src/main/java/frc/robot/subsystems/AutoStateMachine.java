package frc.robot.subsystems;

import java.lang.reflect.Array;
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
import frc.robot.subsystems.Intake.IntakeStates;

public class AutoStateMachine extends SubsystemBase {
    public enum AutoStates {
    StateInit,
    StateFollowingPath,
    StateIdleToFollowingPath
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
    m_Chooser.setDefaultOption("Bottom Side Shooting", "BSideShooting");
    m_Chooser.addOption("Top Side Shooting", "TSideShooting");
    m_Chooser.addOption("Test", "Straight");
    final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    try {
        AutoBuilder.configure(()->m_rc.drivetrain.getState().Pose, (pose) -> m_rc.drivetrain.resetPose(pose.rotateBy(Rotation2d.fromDegrees(0))), ()->m_rc.drivetrain.getState().Speeds,
         (speeds, feedforward) -> m_rc.drivetrain.setControl(m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(new ChassisSpeeds(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,-speeds.omegaRadiansPerSecond), 0.02)).withWheelForceFeedforwardsX(feedforward.robotRelativeForcesXNewtons()).withWheelForceFeedforwardsY(feedforward.robotRelativeForcesYNewtons())),
         new PPHolonomicDriveController(new PIDConstants(.7,0,0), new PIDConstants(.5,0,0)),
         RobotConfig.fromGUISettings(),
         ()->DriverStation.getAlliance().orElse(Alliance.Blue)== Alliance.Red,this
         
         );
    } catch (Exception e) {
        // TODO: handle exception
        System.err.println("AUTO ABORT DUE  TO IO ERROR!");
    }

    // m_Chooser.addOption("B Auto",autoPeriodicStates.MidScoreAutoV2);  // 
    m_RobotContainer = m_rc;
    new EventTrigger("DeployIntake").onTrue(new InstantCommand( () -> m_rc.m_intake.requestState(IntakeStates.StateIntaking)) );
    try {
        autoMap.put("BSideShooting", new PathPlannerPath[]{ PathPlannerPath.fromPathFile("BTrenchToMid"), 
            PathPlannerPath.fromPathFile("BMidToShooting"), 
            PathPlannerPath.fromPathFile("BShootingToDepot")
            });
       // autoMap.get()
       autoMap.put("TSideShooting", new PathPlannerPath[]{ PathPlannerPath.fromPathFile("TTrenchToMid"), 
            PathPlannerPath.fromPathFile("TMidToShooting"), 
            PathPlannerPath.fromPathFile("TShootingToDepot")
            });
        autoMap.put("Straight", new PathPlannerPath[] { PathPlannerPath.fromPathFile("Straight")});
        
        
    } catch (Exception e) {
        // TODO: handle exception
        System.err.print("chud auto fail!");
    }

  }
  @Override
  public void periodic (){
    if(!DriverStation.isAutonomousEnabled()){

        return;
    }
    switch (m_currentState) {
        case StateInit:
            chooser_val = m_Chooser.getSelected();
            m_requestedState=AutoStates.StateIdleToFollowingPath;
            m_RobotContainer.drivetrain.resetGyro();
            
            break;
    
        case StateFollowingPath:
            if (FollowPathCommand.isFinished()){
                PathNumber++;
                m_requestedState = AutoStates.StateIdleToFollowingPath;
            }
            break;

        case StateIdleToFollowingPath:
            PathToFollow = autoMap.get(chooser_val)[PathNumber];
            FollowPathCommand = AutoBuilder.followPath(PathToFollow);
            CommandScheduler.getInstance().schedule(FollowPathCommand);
            m_requestedState=AutoStates.StateFollowingPath;
            m_currentState=AutoStates.StateFollowingPath;
            break;
    }
    m_currentState = m_requestedState;// temp
  }
}
