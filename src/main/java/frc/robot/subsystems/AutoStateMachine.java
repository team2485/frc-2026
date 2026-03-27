package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.nio.file.Path;
import java.util.TreeMap;

import com.ctre.phoenix6.controls.NeutralOut;
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
import frc.robot.subsystems.Windexer.WindexerStates;
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
    public AutoStates m_currentState = AutoStates.StateInit;
    public AutoStates m_requestedState = AutoStates.StateInit;
    public Command FollowPathCommand;

    public RobotContainer m_RobotContainer;
    public int PathNumber = 0;
    TreeMap<String, PathPlannerPath[]> autoMap = new TreeMap<>();
    public SendableChooser<String> m_Chooser = new SendableChooser<>();

    String chooser_val;

    public AutoStateMachine(RobotContainer m_rc) {
        Shuffleboard.getTab("Autos").add(m_Chooser);
        // m_Chooser.addOption("Outpost Side Shooting", "BSideShooting"); // outpost
        // m_Chooser.addOption("Depot Shooting", "TSideShooting"); // depot
        // m_Chooser.addOption("Test", "Straight");
        m_Chooser.setDefaultOption("Outpost Side Shoot", "BSideSimple");
        m_Chooser.addOption("Depot Side Shoot", "TopSideSimple");
        m_Chooser.addOption("Mid Auto", "TSideMid");
        m_Chooser.addOption("Topside Shoot to Mid", "TSideShootToMid");
        m_Chooser.addOption("Brisket", "Brisket");

        // m_Chooser.addOption("Test2", "MoveAndShoot");

        final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
        try {

            AutoBuilder
                    .configure(
                            () -> new Pose2d(m_rc.m_drivetrain.getState().Pose.getTranslation(),
                                    m_rc.m_drivetrain.getState().Pose.getRotation()),
                            (pose) -> m_rc.m_drivetrain.fakeResetPose(pose.rotateBy(Rotation2d.fromDegrees(180))),
                            () -> m_rc.m_drivetrain.getState().Speeds,
                            (speeds, feedforward) -> m_rc.m_drivetrain
                                    .setControl(
                                            m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(
                                                    new ChassisSpeeds(speeds.vxMetersPerSecond,
                                                            speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond),
                                                    0.02))), // .withWheelForceFeedforwardsX(feedforward.robotRelativeForcesXNewtons()).withWheelForceFeedforwardsY(feedforward.robotRelativeForcesYNewtons())),
                            new PPHolonomicDriveController(new PIDConstants(.9, 0, 0), new PIDConstants(1, 0.0, 0)),
                            RobotConfig.fromGUISettings(),
                            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, this

                    );
        } catch (Exception e) {
            System.err.println("AUTO ABORT DUE TO IO ERROR!");
        }

        // m_Chooser.addOption("B Auto",autoPeriodicStates.MidScoreAutoV2); //
        m_RobotContainer = m_rc;
        new EventTrigger("DeployIntake").onTrue(new InstantCommand(() -> {
            m_rc.m_intake.requestState(IntakeStates.StateIntaking);
            ballCountEstimate += 24;
        }));
        new EventTrigger("HoodDown").onTrue(new InstantCommand(() -> {
            m_rc.m_angler.requestState(AnglerStates.StateZero);
        }));
        new EventTrigger("Shoot").onTrue(new InstantCommand(() -> startShooting()));
        new EventTrigger("Wait").onTrue(new InstantCommand(() -> waitTime()));
        //new EventTrigger("Jiggle").onTrue(new InstantCommand(() -> ));

        try {
            autoMap.put("TSideMid", new PathPlannerPath[] {
                    PathPlannerPath.fromPathFile("TSideTrenchShoot"),
                    PathPlannerPath.fromPathFile("TSideShootMid"),
                    PathPlannerPath.fromPathFile("TSideMidToZone"),
                    PathPlannerPath.fromPathFile("TSideZoneToMid"),
                    PathPlannerPath.fromPathFile("TSideZoneToDepot") });

            autoMap.put("BSideShooting", new PathPlannerPath[] {
                    PathPlannerPath.fromPathFile("BTrenchToMid"),
                    PathPlannerPath.fromPathFile("BMidToShooting"),
                    PathPlannerPath.fromPathFile("BShootingToDepot")
            });
            autoMap.put("MoveAndShoot", new PathPlannerPath[] {
                    PathPlannerPath.fromPathFile("MoveAndShoot"),
                    PathPlannerPath.fromPathFile("ShootAndMoveback")
            });
            autoMap.put("BSideSimple", new PathPlannerPath[] {
                    PathPlannerPath.fromPathFile("BSideTrenchShoot"),
                    PathPlannerPath.fromPathFile("BSideTrenchToDepot"),
                    PathPlannerPath.fromPathFile("BSideDepotShoot")
            });
            // autoMap.get()
            autoMap.put("TSideShooting", new PathPlannerPath[] { PathPlannerPath.fromPathFile("TTrenchToMid"),
                    PathPlannerPath.fromPathFile("TMidToShooting"),
                    PathPlannerPath.fromPathFile("TShootingToDepot"),
                    PathPlannerPath.fromPathFile("TSideIntakeToShoot")
            });
            // TSideTrenchShoot TSideTrenchShootToIntake
            autoMap.put("Straight", new PathPlannerPath[] { PathPlannerPath.fromPathFile("Straight") });

            autoMap.put("TopSideSimple", new PathPlannerPath[] {
                    PathPlannerPath.fromPathFile("TSideTrenchShoot"),
                    PathPlannerPath.fromPathFile("TSideTrenchShootToIntake"),
                    PathPlannerPath.fromPathFile("TSideIntakeToShoot"),
                    PathPlannerPath.fromPathFile("TSideShootToTrench")
            });
            autoMap.put("TSideShootToMid", new PathPlannerPath[] {
                    PathPlannerPath.fromPathFile("TSideTrenchShoot"),
                    PathPlannerPath.fromPathFile("TSideShootMid"),
                    PathPlannerPath.fromPathFile("TSideMidToZone"),
                    PathPlannerPath.fromPathFile("TSideZoneToMid"),
                    PathPlannerPath.fromPathFile("TSideZoneToDepot") });
            autoMap.put("Brisket", new PathPlannerPath[] {
                    PathPlannerPath.fromPathFile("Brisket-Mid"),
                    PathPlannerPath.fromPathFile("Brisket-Shoot") });
        } catch (Exception e) {
            System.err.println("chud auto fail!");
        }

    }

    int ballCountEstimate = 40; // was 8, changed for brisket auto
    boolean shootingRequested = false;
    double startShootingTime = -1;
    double waitStartTime = -1;

    @Override
    public void periodic() {
        if (!DriverStation.isAutonomousEnabled()) {
            m_currentState = AutoStates.StateInit;
            m_requestedState = AutoStates.StateInit;
            return;
        }
        switch (m_currentState) {
            case StateInit:
                chooser_val = m_Chooser.getSelected();
                m_requestedState = AutoStates.StateIdleToFollowingPath;
                m_currentState = AutoStates.StateIdleToFollowingPath;
                break;

            case StateFollowingPath:
                if (FollowPathCommand.isFinished()) {

                    if (PathNumber == 0 && !m_Chooser.getSelected().equals("Brisket")) {
                        m_requestedState = AutoStates.IdleToShooting;
                        // PathNumber++;
                    }

                    if (m_requestedState == AutoStates.IdleToShooting || m_requestedState == AutoStates.StateWait) {

                        if (m_Chooser.getSelected().equals("TSideMid")) {
                            if ((int) (DriverStation.getMatchTime()) < 2 && PathNumber == 2) {
                                PathNumber += 2;
                            } else {
                                PathNumber++;
                            }
                        } else {
                            PathNumber++;
                        }
                        m_currentState = m_requestedState;
                    } else {
                        PathNumber++;

                        m_requestedState = AutoStates.StateIdleToFollowingPath;
                        m_currentState = m_requestedState;

                    }
                }
                break;
            case StateWait:

                if (waitStartTime == -1)
                    waitStartTime = System.currentTimeMillis();
                if (System.currentTimeMillis() - waitStartTime > 2500) {

                    m_requestedState = AutoStates.StateIdleToFollowingPath;
                    waitStartTime = -1;
                }

                break;
            case StateIdleToFollowingPath:
                m_RobotContainer.tracker.requestState(TargetingStates.StateDriverControlled);
                m_RobotContainer.m_feeder.requestState(FeederStates.StateOff);
                // m_RobotContainer.m_angler.requestState(AnglerStates.StateOff);
                m_RobotContainer.m_shooter.requestState(ShooterStates.StateOff);

                // CommandScheduler.getInstance().schedule(m_RobotContainer.m_drivetrain.applyRequest(()
                // -> new SwerveRequest.SwerveDriveBrake()));
                CommandScheduler.getInstance().cancelAll();
                if (!(autoMap.get(chooser_val).length - 1 >= PathNumber)) {
                    break;
                }
                PathToFollow = autoMap.get(chooser_val)[PathNumber];
                if (PathToFollow == null) {
                    break;
                }
                if (PathNumber == 0) {
                    if (DriverStation.getAlliance().get() == Alliance.Blue) {

                        m_RobotContainer.m_drivetrain.resetPose(PathToFollow.getStartingHolonomicPose().get());

                    } else {
                        m_RobotContainer.m_drivetrain
                                .resetPose(PathToFollow.flipPath().getStartingHolonomicPose().get());

                    }

                }

                FollowPathCommand = AutoBuilder.followPath(PathToFollow);

                CommandScheduler.getInstance().schedule(FollowPathCommand);
                m_requestedState = AutoStates.StateFollowingPath;
                m_currentState = AutoStates.StateFollowingPath;
                break;
            case IdleToShooting:
                if (FollowPathCommand != null) {
                    FollowPathCommand.cancel();
                }
                // m_RobotContainer.m_drivetrain.applyRequest();
                // m_RobotContainer.tracker.requestState(TargetingStates.StateDriveToAimTransition);

                m_RobotContainer.m_angler.requestState(AnglerStates.StateAuto);
                m_RobotContainer.m_shooter.requestState(ShooterStates.StateShooting);
                if (!m_Chooser.getSelected().equals("Brisket"))
                    m_RobotContainer.m_intake.requestState(IntakeStates.StateRetracted);
                startShootingTime = System.currentTimeMillis();
                m_requestedState = AutoStates.StateShooting;
                m_currentState = m_requestedState;
                break;
            case StateShooting:
                System.out.println("SHOOTING IN AUTO");
                if (System.currentTimeMillis() - startShootingTime > 250) {
                    m_RobotContainer.m_windexer.requestState(WindexerStates.StateFeed);
                    m_RobotContainer.m_feeder.requestState(FeederStates.StateFeeding);
                    if (m_Chooser.getSelected().equals("Brisket")
                            && System.currentTimeMillis() - startShootingTime > 950) {
                                m_RobotContainer.m_intake.requestState(IntakeStates.StateRetracted);
                    }
                }
                if (System.currentTimeMillis() - startShootingTime > ((ballCountEstimate * 100) + 500)) { // times out
                                                                                                          // the
                                                                                                          // shooting

                    m_requestedState = AutoStates.StateIdleToFollowingPath;
                    m_RobotContainer.m_angler.requestState(AnglerStates.StateZero);

                }
                if (m_RobotContainer.tracker.targetLocked == false
                        && System.currentTimeMillis() - startShootingTime > 5 * 1000) { // longest it will try to lock
                                                                                        // on for :)

                    m_requestedState = AutoStates.StateIdleToFollowingPath;
                    m_RobotContainer.m_angler.requestState(AnglerStates.StateZero);

                }
                m_currentState = m_requestedState;
                break;
            case FollowingToShooting:
                // ??
                break;

        }

        // if(PathNumber == 1) {
        // m_requestedState = AutoStates.IdleToShooting;
        // }
        System.out.println("Current state : " + m_currentState.toString());
        System.out.println("Req state : " + m_requestedState.toString());
        if (m_currentState != AutoStates.StateFollowingPath) {

            m_currentState = m_requestedState; // temp
        }
    }

    public void startShooting() {
        System.out.println("wants to shoot ");
        m_requestedState = AutoStates.IdleToShooting;
    }

    public void waitTime() {

        m_requestedState = AutoStates.StateWait;

    }
}
