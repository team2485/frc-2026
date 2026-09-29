package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.kFieldLengthMeters;
import static frc.robot.Constants.VisionConstants.kFieldWidthMeters;
import static frc.robot.Constants.VisionConstants.kMaxVisionZErrorMeters;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * One PhotonVision camera plus the PhotonLib pose estimator for it.
 *
 * <p>Not a subsystem and not threaded: {@link #pollEstimates()} must be called exactly once per
 * robot loop from the main thread. It drains every frame the camera has published since the last
 * call, so each frame is seen exactly once and carries its own capture timestamp.
 */
public class Vision {
    private final String m_name;
    private final PhotonCamera m_camera;
    private final Transform3d m_robotToCamera;
    /** Null if the AprilTag field layout failed to load; the camera is then silently disabled. */
    private final PhotonPoseEstimator m_photonPoseEstimator;

    public Vision(String cameraName, Transform3d robotToCamera) {
        m_name = cameraName;
        m_camera = new PhotonCamera(cameraName);
        m_robotToCamera = robotToCamera;

        PhotonPoseEstimator estimator = null;
        try {
            AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            estimator = new PhotonPoseEstimator(layout, robotToCamera);
        } catch (Exception e) {
            DriverStation.reportError("Vision " + cameraName + ": failed to load the AprilTag field layout; "
                    + "this camera will not contribute to the pose", e.getStackTrace());
        }
        m_photonPoseEstimator = estimator;
    }

    /**
     * Drains every frame published since the last call and returns one estimate per usable frame,
     * oldest first. Uses the coprocessor multi-tag solve when the frame has one, otherwise the
     * average-best-target single-tag solve. Solves that land outside the field or off the floor are
     * dropped here; everything else is left to {@link VisionMeasurementMath#evaluate}.
     */
    public List<VisionEstimate> pollEstimates() {
        if (m_photonPoseEstimator == null) {
            return List.of();
        }
        List<VisionEstimate> out = new ArrayList<>();
        for (PhotonPipelineResult result : m_camera.getAllUnreadResults()) {
            if (!result.hasTargets()) {
                continue;
            }
            Optional<EstimatedRobotPose> maybeEstimate = m_photonPoseEstimator.estimateCoprocMultiTagPose(result)
                    .or(() -> m_photonPoseEstimator.estimateAverageBestTargetsPose(result));
            if (maybeEstimate.isEmpty()) {
                continue;
            }
            EstimatedRobotPose estimate = maybeEstimate.get();
            Pose3d pose = estimate.estimatedPose;
            if (pose.getX() < 0.0 || pose.getX() > kFieldLengthMeters
                    || pose.getY() < 0.0 || pose.getY() > kFieldWidthMeters
                    || Math.abs(pose.getZ()) > kMaxVisionZErrorMeters) {
                continue;
            }

            int numTags = estimate.targetsUsed.size();
            double distanceSum = 0.0;
            double maxAmbiguity = 0.0;
            for (PhotonTrackedTarget target : estimate.targetsUsed) {
                distanceSum += target.getBestCameraToTarget().getTranslation().getNorm();
                maxAmbiguity = Math.max(maxAmbiguity, target.getPoseAmbiguity());
            }
            double avgDistance = numTags > 0 ? distanceSum / numTags : Double.POSITIVE_INFINITY;
            boolean multiTag = estimate.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

            out.add(new VisionEstimate(
                    m_name,
                    pose.toPose2d(),
                    estimate.timestampSeconds,
                    result.metadata.getLatencyMillis() / 1000.0,
                    numTags,
                    multiTag,
                    avgDistance,
                    maxAmbiguity));
        }
        return out;
    }

    public String getName() {
        return m_name;
    }

    public boolean isConnected() {
        return m_camera.isConnected();
    }

    /** For simulation wiring only. */
    public PhotonCamera getCamera() {
        return m_camera;
    }

    /** For simulation wiring only. */
    public Transform3d getRobotToCamera() {
        return m_robotToCamera;
    }
}
