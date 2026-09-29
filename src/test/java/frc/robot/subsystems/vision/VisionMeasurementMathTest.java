package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.kMaxOmegaForVisionRadPerSec;
import static frc.robot.Constants.VisionConstants.kMotionStdDevGain;
import static frc.robot.Constants.VisionConstants.kVisionMaxDistanceMeters;
import static frc.robot.Constants.VisionConstants.kVisionThetaStdDevRadians;
import static frc.robot.Constants.VisionConstants.kXYStdDevCoefficient;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.vision.VisionMeasurementMath.RejectReason;

class VisionMeasurementMathTest {
    private static final double kEps = 1e-9;

    private static VisionEstimate estimate(double timestamp, int numTags, boolean multiTag, double avgDist,
            double ambiguity) {
        return new VisionEstimate("TestCam", new Pose2d(1, 2, Rotation2d.kZero), timestamp, 0.03, numTags,
                multiTag, avgDist, ambiguity);
    }

    /* ---------------- stdDevs ---------------- */

    @Test
    void stationarySingleTagAtOneMetreGivesBaseStdDev() {
        var std = VisionMeasurementMath.stdDevs(estimate(1.0, 1, false, 1.0, 0.0), new ChassisSpeeds(), 0.05);
        assertEquals(kXYStdDevCoefficient, std.get(0, 0), kEps);
        assertEquals(kXYStdDevCoefficient, std.get(1, 0), kEps);
        assertEquals(kVisionThetaStdDevRadians, std.get(2, 0), kEps);
    }

    @Test
    void twoTagsAtTwoMetresScaleByDistanceSquaredOverCount() {
        var std = VisionMeasurementMath.stdDevs(estimate(1.0, 2, true, 2.0, 0.0), new ChassisSpeeds(), 0.0);
        assertEquals(kXYStdDevCoefficient * 4.0 / 2.0, std.get(0, 0), kEps);
        assertEquals(kXYStdDevCoefficient * 4.0 / 2.0, std.get(1, 0), kEps);
    }

    @Test
    void motionInflatesOnlyTheAxisOfTravel() {
        var std = VisionMeasurementMath.stdDevs(estimate(1.0, 1, false, 1.0, 0.0), new ChassisSpeeds(2.0, 0, 0), 0.1);
        assertEquals(kXYStdDevCoefficient + 2.0 * 0.1 * kMotionStdDevGain, std.get(0, 0), kEps);
        assertEquals(kXYStdDevCoefficient, std.get(1, 0), kEps);
    }

    @Test
    void robotForwardAtNinetyDegreesHeadingInflatesFieldY() {
        ChassisSpeeds field = ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(2.0, 0, 0),
                Rotation2d.fromDegrees(90));
        var std = VisionMeasurementMath.stdDevs(estimate(1.0, 1, false, 1.0, 0.0), field, 0.1);
        assertEquals(kXYStdDevCoefficient, std.get(0, 0), 1e-6);
        assertEquals(kXYStdDevCoefficient + 0.2 * kMotionStdDevGain, std.get(1, 0), 1e-6);
    }

    @Test
    void negativeAgeDoesNotShrinkStdDev() {
        var std = VisionMeasurementMath.stdDevs(estimate(1.0, 1, false, 1.0, 0.0), new ChassisSpeeds(2.0, 0, 0), -0.1);
        assertEquals(kXYStdDevCoefficient, std.get(0, 0), kEps);
    }

    /* ---------------- evaluate ---------------- */

    @Test
    void goodFrameIsAccepted() {
        assertEquals(RejectReason.ACCEPTED,
                VisionMeasurementMath.evaluate(estimate(10.0, 2, true, 2.0, 0.0), 10.05, 0.0, 0.0));
    }

    @Test
    void frameBeforeResetIsRejected() {
        assertEquals(RejectReason.BEFORE_RESET,
                VisionMeasurementMath.evaluate(estimate(9.0, 2, true, 2.0, 0.0), 10.05, 9.5, 0.0));
    }

    @Test
    void frameFarInFutureIsRejected() {
        assertEquals(RejectReason.FUTURE,
                VisionMeasurementMath.evaluate(estimate(10.10, 2, true, 2.0, 0.0), 10.0, 0.0, 0.0));
    }

    @Test
    void frameSlightlyInFutureIsAcceptedWithinTolerance() {
        assertEquals(RejectReason.ACCEPTED,
                VisionMeasurementMath.evaluate(estimate(10.01, 2, true, 2.0, 0.0), 10.0, 0.0, 0.0));
    }

    @Test
    void staleFrameIsRejected() {
        assertEquals(RejectReason.STALE,
                VisionMeasurementMath.evaluate(estimate(9.0, 2, true, 2.0, 0.0), 10.0, 0.0, 0.0));
    }

    @Test
    void farTagsAreRejected() {
        assertEquals(RejectReason.TOO_FAR,
                VisionMeasurementMath.evaluate(estimate(10.0, 2, true, kVisionMaxDistanceMeters + 1.0, 0.0),
                        10.05, 0.0, 0.0));
    }

    @Test
    void ambiguousSingleTagIsRejected() {
        assertEquals(RejectReason.AMBIGUOUS,
                VisionMeasurementMath.evaluate(estimate(10.0, 1, false, 2.0, 0.5), 10.05, 0.0, 0.0));
    }

    @Test
    void ambiguousMultiTagIsAccepted() {
        assertEquals(RejectReason.ACCEPTED,
                VisionMeasurementMath.evaluate(estimate(10.0, 2, true, 2.0, 0.3), 10.05, 0.0, 0.0));
    }

    @Test
    void spinningIsRejected() {
        assertEquals(RejectReason.ROTATING_TOO_FAST,
                VisionMeasurementMath.evaluate(estimate(10.0, 2, true, 2.0, 0.0), 10.05, 0.0,
                        kMaxOmegaForVisionRadPerSec + 1.0));
    }

    /* ---------------- projectForward ---------------- */

    @Test
    void projectsForwardAlongHeadingZero() {
        Pose2d out = VisionMeasurementMath.projectForward(new Pose2d(1, 1, Rotation2d.kZero),
                new ChassisSpeeds(1.0, 0, 0), 0.1);
        assertEquals(1.1, out.getX(), kEps);
        assertEquals(1.0, out.getY(), kEps);
        assertEquals(0.0, out.getRotation().getRadians(), kEps);
    }

    @Test
    void projectsForwardAlongHeadingNinety() {
        Pose2d out = VisionMeasurementMath.projectForward(new Pose2d(1, 1, Rotation2d.fromDegrees(90)),
                new ChassisSpeeds(1.0, 0, 0), 0.1);
        assertEquals(1.0, out.getX(), 1e-9);
        assertEquals(1.1, out.getY(), 1e-9);
    }

    @Test
    void projectsRotationAsArcNotChord() {
        // 1 m/s forward while turning at pi rad/s for 0.5 s: quarter turn, both x and y end at 1/pi.
        Pose2d out = VisionMeasurementMath.projectForward(Pose2d.kZero, new ChassisSpeeds(1.0, 0, Math.PI), 0.5);
        assertEquals(90.0, out.getRotation().getDegrees(), 1e-9);
        assertEquals(1.0 / Math.PI, out.getX(), 1e-9);
        assertEquals(1.0 / Math.PI, out.getY(), 1e-9);
    }

    @Test
    void negativeDtDoesNotProjectBackward() {
        Pose2d in = new Pose2d(1, 1, Rotation2d.kZero);
        Pose2d out = VisionMeasurementMath.projectForward(in, new ChassisSpeeds(1.0, 0, 0), -0.1);
        assertEquals(in, out);
    }
}
