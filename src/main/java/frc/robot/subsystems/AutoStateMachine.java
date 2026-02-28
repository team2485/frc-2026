package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.TreeMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.IntakeStates;

public class AutoStateMachine extends SubsystemBase {
    public enum AutoStates {
    StateInit,
    StateFollowingPath,
    StateIdleToFollowingPath
    }
  public PathPlannerPath PathToFollow;
  public AutoStates m_currentState;
  public AutoStates m_requestedState;
  public Command FollowPathCommand;

  public RobotContainer m_RobotContainer;
  public int PathNumber = 0;
  TreeMap<String, PathPlannerPath[]> autoMap = new TreeMap<>();
  public SendableChooser<String> m_Chooser = new SendableChooser<>();
 
  String chooser_val;

  public AutoStateMachine (RobotContainer m_rc){
    m_Chooser.setDefaultOption("Bottom Side Shooting", "BSideShooting");
    m_Chooser.addOption("Top Side Shooting", "TSideShooting");
    
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
        
        
    } catch (Exception e) {
        // TODO: handle exception
        System.err.print("chud auto fail!");
    }

  }
  @Override
  public void periodic (){
    switch (m_currentState) {
        case StateInit:
            chooser_val = m_Chooser.getSelected();
            m_requestedState=AutoStates.StateIdleToFollowingPath;
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
