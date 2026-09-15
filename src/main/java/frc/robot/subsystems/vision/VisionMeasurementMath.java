package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.kMaxOmegaForVisionRadPerSec;
import static frc.robot.Constants.VisionConstants.kMaxSingleTagAmbiguity;
import static frc.robot.Constants.VisionConstants.kMaxVisionAgeSeconds;
import static frc.robot.Constants.VisionConstants.kMaxVisionFutureSeconds;
import static frc.robot.Constants.VisionConstants.kMotionStdDevGain;
import static frc.robot.Constants.VisionConstants.kVisionMaxDistanceMeters;
import static frc.robot.Constants.VisionConstants.kVisionThetaStdDevRadians;
import static frc.robot.Constants.VisionConstants.kXYStdDevCoefficient;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Pure math for fusing camera frames into the pose estimator: accept/reject gating, per-frame
 * measurement trust, and forward projection of a stale vision pose. No hardware or NetworkTables,
 * so it runs under plain JUnit.
 */
public final class VisionMeasurementMath {
    private VisionMeasurementMath() {
    }

    /** Why a frame was or was not fused. Logged per camera as {@code Vision/<cam>/RejectReason}. */
    public enum RejectReason {
        ACCEPTED,
        /** Captured before the last pose reset; fusing it would drag the reset pose back. */
        BEFORE_RESET,
        /** Timestamp further ahead of "now" than time-sync jitter allows. */
        FUTURE,
        /** Older than {@code kMaxVisionAgeSeconds} at fusion time. */
        STALE,
        /** Tags further away on average than {@code kVisionMaxDistanceMeters}. */
        TOO_FAR,
        /** Single-tag solve whose ambiguity exceeds {@code kMaxSingleTagAmbiguity}. */
        AMBIGUOUS,
        /** Robot was spinning faster than {@code kMaxOmegaForVisionRadPerSec} (rotation blur). */
        ROTATING_TOO_FAST
    }

    /**
     * Decides whether a frame should be fused.
     *
     * @param est                   the frame
     * @param nowSeconds            current FPGA time
     * @param resetTimestampSeconds FPGA time of the last pose reset
     * @param omegaRadPerSec        current chassis angular velocity
     */
    public static RejectReason evaluate(VisionEstimate est, double nowSeconds, double resetTimestampSeconds,
            double omegaRadPerSec) {
        if (est.timestampSeconds() < resetTimestampSeconds) {
            return RejectReason.BEFORE_RESET;
        }
        double age = nowSeconds - est.timestampSeconds();
        if (age < -kMaxVisionFutureSeconds) {
            return RejectReason.FUTURE;
        }
        if (age > kMaxVisionAgeSeconds) {
            return RejectReason.STALE;
        }
        if (est.avgTagDistanceMeters() > kVisionMaxDistanceMeters) {
            return RejectReason.TOO_FAR;
        }
        if (!est.multiTag() && est.maxAmbiguity() > kMaxSingleTagAmbiguity) {
            return RejectReason.AMBIGUOUS;
        }
        if (Math.abs(omegaRadPerSec) > kMaxOmegaForVisionRadPerSec) {
            return RejectReason.ROTATING_TOO_FAST;
        }
        return RejectReason.ACCEPTED;
    }

    /**
     * Measurement std devs (x, y, theta) for one frame.
     *
     * <p>The base X/Y trust scales with average tag distance squared over tag count. On top of that,
     * each field axis gets an extra term equal to the distance the robot travelled along that axis
     * while the frame was in flight ({@code |v_axis| * age * gain}), so a frame from the
     * higher-latency camera is trusted less along the direction of travel and normally when the robot
     * is stationary. Theta is huge so the gyro owns heading.
     *
     * @param est                 the frame
     * @param fieldRelativeSpeeds current chassis speeds in the field frame
     * @param ageSeconds          now minus the frame's capture time (negative values are treated as 0)
     */
    public static Matrix<N3, N1> stdDevs(VisionEstimate est, ChassisSpeeds fieldRelativeSpeeds, double ageSeconds) {
        double distFactor = est.avgTagDistanceMeters() * est.avgTagDistanceMeters() / Math.max(est.numTags(), 1);
        double xy = kXYStdDevCoefficient * distFactor;
        double age = Math.max(ageSeconds, 0.0);
        double x = xy + Math.abs(fieldRelativeSpeeds.vxMetersPerSecond) * age * kMotionStdDevGain;
        double y = xy + Math.abs(fieldRelativeSpeeds.vyMetersPerSecond) * age * kMotionStdDevGain;
        double theta = kVisionThetaStdDevRadians
                + Math.abs(fieldRelativeSpeeds.omegaRadiansPerSecond) * age * kMotionStdDevGain;
        return VecBuilder.fill(x, y, theta);
    }

    /**
     * Moves a pose captured {@code dtSeconds} ago to "now" along the current robot-relative velocity
     * (constant-velocity twist, so a turn is an arc rather than a chord). Negative {@code dtSeconds}
     * leaves the pose unchanged.
     */
    public static Pose2d projectForward(Pose2d capturePose, ChassisSpeeds robotRelativeSpeeds, double dtSeconds) {
        double dt = Math.max(dtSeconds, 0.0);
        return capturePose.exp(new Twist2d(
                robotRelativeSpeeds.vxMetersPerSecond * dt,
                robotRelativeSpeeds.vyMetersPerSecond * dt,
                robotRelativeSpeeds.omegaRadiansPerSecond * dt));
    }
}
