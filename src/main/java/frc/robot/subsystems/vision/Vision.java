package frc.robot.subsystems.vision;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Predicate;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.VisionConstants.*;

public class Vision implements Runnable {

    class WhitelistFilter implements Predicate<PhotonTrackedTarget> {
        int[] whitelist;

        public WhitelistFilter(int[] allowedTags) {
            whitelist = allowedTags;
        }

        @Override
        public boolean test(PhotonTrackedTarget t) {
            // TODO Auto-generated method stub
            // throw new UnsupportedOperationException("Unimplemented method 'test'");
            for (int i : whitelist) {

                if (t.getFiducialId() == i) {
                    return false;

                }

            }
            return true;

        }

    }

    class TagFilter implements Predicate<PhotonTrackedTarget> {
        double maxDist = 2;

        public TagFilter(double maxDist) {
            this.maxDist = maxDist;
        }

        @Override
        public boolean test(PhotonTrackedTarget t) {
            // TODO Auto-generated method stub
            // throw new UnsupportedOperationException("Unimplemented method 'test'");

            if (!DriverStation.isEnabled()) {
                if (t.getPoseAmbiguity() > 0.02) {
                    return true;
                }
                return false;

            }

            if (t.getPoseAmbiguity() > 0.02) {
                return true;
            }
            Transform3d vector = t.getBestCameraToTarget();
            if (vector.getTranslation().getNorm() > maxDist) {

                return true;

            }
            return false;

        }
    }

    // PhotonVision class that takes vision measurements from camera and
    // triangulates position on field
    private final PhotonPoseEstimator m_photonPoseEstimator;
    // private final PhotonPoseEstimator m_estimatorWithError;
    // creates new PhotonCamera object for camera
    private final PhotonCamera m_camera; // initialize with a USEFUL name;

    // creates a thread-safe object reference since mutliple robot poses could be
    // reported concurrently and conflict
    // lowkey an interesting read, the atomic library gaslights machine instruction
    // algos like compare-and-swap
    // that are instrinsically atomic into working for concurrency
    GenericEntry cameraExists;
    private final AtomicReference<EstimatedRobotPose> m_atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
    // private final AtomicReference<EstimatedRobotPose>
    // m_atomicEstimatedBadRobotPose = new AtomicReference<EstimatedRobotPose>();

    private Pose3d m_badPose;

    public Vision(String cameraName) {
        PhotonPoseEstimator photonPoseEstimator = null;
        // PhotonPoseEstimator estimatorWithError = null;

        // cameraExists = Shuffleboard.getTab("Swerve").add("CameraExists",
        // 0).getEntry();

        this.m_camera = new PhotonCamera(cameraName);
        // var offset = kRobotToCameraLeft;
        try {
            // sets the origin to the blue side every time but flips the tag positions if we
            // are red.
            // var layout = new AprilTagFieldLayout(AprilTagFieldLayout.load),
            // kFieldLengthMeters, kFieldWidthMeters); // attempt to load the
            // AprilTagFieldLayout
            var layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
            // var layout =
            // AprilTagFieldLayout.loadFromResource("C:\\Users\\RoboticsUser\\Desktop\\frc-2026\\src\\main\\java\\frc\\util\\2026-rebuilt-welded.json");
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

            if (m_camera != null) {

                photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        new Transform3d(Translation3d.kZero,
                                new Rotation3d(0, 0, 0)

                        )); // MULTI_TAG_PNP uses all cameras in view for positioning
                // estimatorWithError = new PhotonPoseEstimator(layout,
                // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                // offset
                // );
            }
        } catch (Exception e) {
            DriverStation.reportError("Path: ", e.getStackTrace()); // can't estimate poses without known tag positions
            photonPoseEstimator = null;
        }
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        // estimatorWithError.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        this.m_photonPoseEstimator = photonPoseEstimator;
        // this.m_estimatorWithError = estimatorWithError;
    }

    public Pose3d getPoseWithAmbiguity() {

        return m_badPose;

    }

    @Override
    public void run() {

        if (m_photonPoseEstimator != null && m_camera != null) { // environment and camera must be initialized properly
            var photonResults = m_camera.getLatestResult(); // continuously get latest camera reading

                // photonResults.targets.removeIf(n->(n.getPoseAmbiguity() >0.0) ); // old
                // filter :(

                // new filter:
                // if(isReefCamera){

                // photonResults.targets.removeIf(new TagFilter(kVisionMaxDistanceMeters));
                // photonResults.targets.removeIf(new
                // WhitelistFilter(PoseEstimation.getFieldConstants().getReefTagIds()));

                // } else{

                // photonResults.targets.removeIf(new TagFilter(kVisionMaxDistanceMeters));
                // photonResults.targets.removeIf(new
                // WhitelistFilter(PoseEstimation.getFieldConstants().getReefTagIds()));

                // }

                // m_estimatorWithError.update(photonResults).ifPresent(m_badPose -> {
                // var estimatedPose = m_badPose.estimatedPose;

                // //SmartDashboard.putBoolean("Camera Positioned For Auto", true);
                // //cameraExists.setDouble(photonResults.targets.get(0).getBestCameraToTarget().getX());
                // //cameraExists.setDouble(photonResults.getMultiTagResult().estimatedPose.best.getX());

                // if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= kFieldLengthMeters
                // && estimatedPose.getY() > 0.0
                // && estimatedPose.getY() <= kFieldWidthMeters) { // idiot check

                // m_atomicEstimatedBadRobotPose.set(m_badPose);
                // }}
                // );
            
            photonResults = m_camera.getLatestResult();
            // SmartDashboard.putString("Visible Tag", String.format("(%.3f, %.3f) %.2f
            // radians", photonResults.targets.get(0).getBestCameraToTarget().getX(),
            // photonResults.targets.get(0).getBestCameraToTarget().getY(),
            // photonResults.targets.get(0).getBestCameraToTarget().getRotation()));
            if (photonResults.hasTargets()) {
                // && ((photonResults.targets.size() > 1) ||
                // (photonResults.targets.get(0).getBestCameraToTarget().getX() < 1.75 &&
                // photonResults.targets.get(0).getPoseAmbiguity() < .2))) { // need accurate
                // readings
                // photonResults.targets.removeIf(n->(n.getFiducialId() != 3 &&
                // n.getFiducialId() != 4 && n.getFiducialId() != 7 && n.getFiducialId() != 8));
                /*                         || (photonResults.targets.get(0).getBestCameraToTarget().getX() < 1.75 &&
                                photonResults.targets.get(0).getPoseAmbiguity() < .2) && ((photonResults.targets.size() > 1) */
                if (photonResults.targets.size() > 0) {
                    m_photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
                        var estimatedPose = estimatedRobotPose.estimatedPose;

                        SmartDashboard.putBoolean("Camera Positioned For Auto", true);
                        // cameraExists.setDouble(photonResults.targets.get(0).getBestCameraToTarget().getX());
                        // cameraExists.setDouble(photonResults.getMultiTagResult().estimatedPose.best.getX());

                        if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= kFieldLengthMeters
                                && estimatedPose.getY() > 0.0
                                && estimatedPose.getY() <= kFieldWidthMeters) { // idiot check
                            m_atomicEstimatedRobotPose.set(estimatedRobotPose);
                        }
                    });
                }
            } else {
                m_atomicEstimatedRobotPose.set(null);
                // SmartDashboard.putBoolean("Camera Positioned For Auto", false);
            }
                
        }

    }

    public EstimatedRobotPose grabLatestEstimatedPose() {
        return m_atomicEstimatedRobotPose.get();
    }

}
