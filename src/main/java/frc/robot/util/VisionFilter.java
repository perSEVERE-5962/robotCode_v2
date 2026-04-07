package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Pure-math vision pose filter. No WPILib sim deps, fully unit-testable.
 *
 * <p>Rejection checks, distance/velocity std dev scaling, and single-tag pose blending. Cross-team
 * validated: 6/12 top FRC teams implement similar filtering (1540, 5026, 4795, 3636, 118, 360).
 */
public final class VisionFilter {

  // Rejection thresholds
  public static final double MAX_AMBIGUITY = 0.25;
  public static final double MAX_Z_HEIGHT_M = 0.5;
  public static final double FIELD_LENGTH_M = 16.541;
  public static final double FIELD_WIDTH_M = 8.069;
  public static final double FIELD_MARGIN_M = 0.5;
  public static final double MAX_HEADING_DIVERGENCE_DEG = 30.0;
  public static final double MAX_POSE_JUMP_M = 2.0;
  public static final double MAX_ROLL_PITCH_DEG = 12.0;
  public static final double BLEND_DISTANCE_THRESHOLD_M = 3.0;
  public static final double VELOCITY_SCALE_FACTOR = 0.3;
  public static final double DISTANCE_SCALE_DIVISOR = 30.0;

  // Skip pose jump check during early auto
  private static final double AUTO_GRACE_PERIOD_SEC = 2.0;

  public enum RejectionReason {
    ACCEPTED,
    AMBIGUITY,
    Z_HEIGHT,
    ROLL_PITCH,
    FIELD_BOUNDS,
    HEADING_DIVERGENCE,
    POSE_JUMP
  }

  /**
   * Run rejection checks on a vision pose estimate.
   *
   * @param visionPose 3D pose from PhotonVision PnP
   * @param tagCount number of AprilTags used in this estimate
   * @param worstAmbiguity highest ambiguity among targets (0-1)
   * @param gyroHeading current gyro heading for divergence check
   * @param currentPose current fused pose from Kalman filter
   * @param autoElapsedSec seconds since autonomous started (for grace period)
   * @return ACCEPTED or the first check that rejected
   */
  public static RejectionReason evaluate(
      Pose3d visionPose,
      int tagCount,
      double worstAmbiguity,
      Rotation2d gyroHeading,
      Pose2d currentPose,
      double autoElapsedSec) {

    // Ambiguity check (single-tag only, multi-tag PnP doesn't suffer)
    if (tagCount == 1 && worstAmbiguity > MAX_AMBIGUITY) {
      return RejectionReason.AMBIGUITY;
    }

    // Z-height sanity (robot can't fly or be underground)
    if (Math.abs(visionPose.getZ()) > MAX_Z_HEIGHT_M) {
      return RejectionReason.Z_HEIGHT;
    }

    // Roll/pitch sanity (extreme tilt = bad PnP solve)
    double rollDeg = Math.toDegrees(visionPose.getRotation().getX());
    double pitchDeg = Math.toDegrees(visionPose.getRotation().getY());
    if (Math.abs(rollDeg) > MAX_ROLL_PITCH_DEG || Math.abs(pitchDeg) > MAX_ROLL_PITCH_DEG) {
      return RejectionReason.ROLL_PITCH;
    }

    // Field bounds (robot can't be outside the walls)
    Pose2d pose2d = visionPose.toPose2d();
    double x = pose2d.getX();
    double y = pose2d.getY();
    if (x < -FIELD_MARGIN_M
        || x > FIELD_LENGTH_M + FIELD_MARGIN_M
        || y < -FIELD_MARGIN_M
        || y > FIELD_WIDTH_M + FIELD_MARGIN_M) {
      return RejectionReason.FIELD_BOUNDS;
    }

    // Heading divergence (single-tag heading is unreliable, gyro is truth)
    if (tagCount == 1 && gyroHeading != null) {
      double headingDiffDeg = Math.abs(pose2d.getRotation().minus(gyroHeading).getDegrees());
      if (headingDiffDeg > MAX_HEADING_DIVERGENCE_DEG) {
        return RejectionReason.HEADING_DIVERGENCE;
      }
    }

    // Pose jump (can't teleport, but skip during early auto)
    if (currentPose != null && autoElapsedSec > AUTO_GRACE_PERIOD_SEC) {
      double jumpM = pose2d.getTranslation().getDistance(currentPose.getTranslation());
      if (jumpM > MAX_POSE_JUMP_M) {
        return RejectionReason.POSE_JUMP;
      }
    }

    return RejectionReason.ACCEPTED;
  }

  /**
   * Compute final std devs with distance, velocity, and tag-count scaling.
   *
   * @param tagCount number of tags in estimate
   * @param avgDistanceM average distance to visible tags
   * @param robotSpeedMps current robot speed magnitude
   * @param singleTagBase base std devs for single-tag
   * @param multiTagBase base std devs for multi-tag
   * @return scaled std dev matrix [x, y, theta]
   */
  public static Matrix<N3, N1> computeStdDevs(
      int tagCount,
      double avgDistanceM,
      double robotSpeedMps,
      Matrix<N3, N1> singleTagBase,
      Matrix<N3, N1> multiTagBase) {

    Matrix<N3, N1> base;

    if (tagCount == 0) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else if (tagCount == 1) {
      // Single-tag: use translation from base, but heading = infinity
      // Gyro is always better than single-tag heading estimation
      base =
          VecBuilder.fill(
              singleTagBase.get(0, 0), singleTagBase.get(1, 0), Double.POSITIVE_INFINITY);
    } else {
      base = multiTagBase;
    }

    // Distance-squared scaling (photogrammetry: pixel error grows quadratically)
    double distScale = 1.0 + (avgDistanceM * avgDistanceM / DISTANCE_SCALE_DIVISOR);

    // Velocity scaling (motion blur + latency displacement)
    double velScale = 1.0 + (robotSpeedMps * robotSpeedMps * VELOCITY_SCALE_FACTOR);

    // Cap velocity scale to prevent total vision blackout at high speed
    velScale = Math.min(velScale, 5.0);

    double combinedScale = distScale * velScale;

    return VecBuilder.fill(
        base.get(0, 0) * combinedScale,
        base.get(1, 0) * combinedScale,
        base.get(2, 0) * combinedScale);
  }

  /**
   * Compute blend weight for single-tag pose blending. Close tags = high weight (trust tag), far
   * tags = low weight (trust odometry).
   *
   * @param distanceM distance to the tag
   * @return blend weight 0.0-1.0
   */
  public static double computeBlendWeight(double distanceM) {
    if (distanceM > BLEND_DISTANCE_THRESHOLD_M) {
      return 0.0;
    }
    return 1.0 / (1.0 + distanceM * distanceM);
  }

  /**
   * Blend single-tag pose with current fused pose. Uses tag for translation, gyro for heading.
   *
   * @param fusedPose current Kalman filter pose (odometry + previous vision)
   * @param singleTagPose pose from single AprilTag estimate
   * @param blendWeight 0.0 = all fused, 1.0 = all single-tag
   * @return blended pose with interpolated translation and fused heading
   */
  public static Pose2d blendPose(Pose2d fusedPose, Pose2d singleTagPose, double blendWeight) {
    Translation2d blendedTranslation =
        fusedPose.getTranslation().interpolate(singleTagPose.getTranslation(), blendWeight);
    // Always use fused heading (gyro-backed), never single-tag heading
    return new Pose2d(blendedTranslation, fusedPose.getRotation());
  }

  private VisionFilter() {}
}
