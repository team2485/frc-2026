package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * One robot-pose estimate produced from a single camera frame.
 *
 * @param cameraName             PhotonVision camera name the frame came from
 * @param pose                   field-relative robot pose solved from this frame
 * @param timestampSeconds       capture time of the frame, in the RIO's FPGA time base
 * @param pipelineLatencySeconds coprocessor capture-to-publish latency (logging only; fusion uses the
 *                               frame's age at fusion time, which also includes NT transport)
 * @param numTags                number of AprilTags used in the solve
 * @param multiTag               true if the pose came from the multi-tag PnP solve, false for the
 *                               single-tag fallback
 * @param avgTagDistanceMeters   mean camera-to-tag distance over the tags used
 * @param maxAmbiguity           largest single-tag pose ambiguity over the tags used (only meaningful
 *                               when {@code multiTag} is false)
 */
public record VisionEstimate(
        String cameraName,
        Pose2d pose,
        double timestampSeconds,
        double pipelineLatencySeconds,
        int numTags,
        boolean multiTag,
        double avgTagDistanceMeters,
        double maxAmbiguity) {
}
