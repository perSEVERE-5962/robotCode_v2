package frc.robot.subsystems.swervedrive;

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
 * <p>10-gate rejection, distance/velocity std dev scaling, and single-tag pose blending.
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

  // New gate thresholds
  public static final double MAX_GYRO_RATE_DEG_PER_SEC = 120.0;
  public static final double STALE_THRESHOLD_STOPPED_SEC = 1.0;
  public static final double STALE_THRESHOLD_FAST_SEC = 0.3;
  public static final double STALE_SPEED_CUTOFF_MPS = 2.0;
  public static final double MAX_SINGLE_TAG_DISTANCE_M = 5.0;

  // Cameras at different frame rates can desync; only fuse frames within this window
  public static final double FRAME_RECENCY_THRESHOLD_SEC = 0.25;

  // Being spun by an opponent: high gyro rate but low wheel speed
  public static final double DEFENSE_GYRO_RATE_DPS = 150.0;
  public static final double DEFENSE_MAX_SPEED_MPS = 1.0;
  public static final double DEFENSE_STD_DEV_SCALE = 0.5;

  // Below this we resolve ambiguity instead of rejecting; above this it's unresolvable
  public static final double AMBIGUITY_HARD_REJECT = 0.8;
  public static final double AMBIGUITY_STD_DEV_INFLATE = 4.0;

  // 2026 REBUILT: blue tags 1-16, red tags 17-32
  private static final int BLUE_TAG_MAX = 16;

  // Skip pose jump check during early auto
  private static final double AUTO_GRACE_PERIOD_SEC = 2.0;

  public enum RejectionReason {
    ACCEPTED,
    AMBIGUITY,
    Z_HEIGHT,
    ROLL_PITCH,
    FIELD_BOUNDS,
    HEADING_DIVERGENCE,
    POSE_JUMP,
    GYRO_RATE,
    OPPOSING_ALLIANCE,
    STALE,
    DISTANCE
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
    return evaluate(
        visionPose,
        tagCount,
        worstAmbiguity,
        gyroHeading,
        currentPose,
        autoElapsedSec,
        0,
        0,
        0,
        false,
        new int[0],
        0);
  }

  /**
   * Run all 10 rejection checks on a vision pose estimate.
   *
   * @param gyroRateDegPerSec current gyro angular velocity (for mid-spin rejection)
   * @param robotSpeedMps current robot speed (for staleness scaling)
   * @param ageSec age of this vision frame in seconds
   * @param isBlueAlliance true if we're on blue alliance
   * @param tagIds fiducial IDs of all tags used in this estimate
   * @param avgTagDistanceM average distance from camera to visible tags
   */
  public static RejectionReason evaluate(
      Pose3d visionPose,
      int tagCount,
      double worstAmbiguity,
      Rotation2d gyroHeading,
      Pose2d currentPose,
      double autoElapsedSec,
      double gyroRateDegPerSec,
      double robotSpeedMps,
      double ageSec,
      boolean isBlueAlliance,
      int[] tagIds,
      double avgTagDistanceM) {
    return evaluate(
        visionPose,
        tagCount,
        worstAmbiguity,
        gyroHeading,
        currentPose,
        autoElapsedSec,
        gyroRateDegPerSec,
        robotSpeedMps,
        ageSec,
        isBlueAlliance,
        tagIds,
        avgTagDistanceM,
        false);
  }

  /** Full version with defense awareness. */
  public static RejectionReason evaluate(
      Pose3d visionPose,
      int tagCount,
      double worstAmbiguity,
      Rotation2d gyroHeading,
      Pose2d currentPose,
      double autoElapsedSec,
      double gyroRateDegPerSec,
      double robotSpeedMps,
      double ageSec,
      boolean isBlueAlliance,
      int[] tagIds,
      double avgTagDistanceM,
      boolean underDefense) {

    // Gate 1: Ambiguity (single-tag only). Between 0.25-0.8 we try to resolve
    // rather than reject, so only hard-reject truly unresolvable estimates.
  //   if (tagCount == 1 && worstAmbiguity > AMBIGUITY_HARD_REJECT) {
  //     return RejectionReason.AMBIGUITY;
  //   }

  //   // Gate 2: Z-height sanity (robot can't fly or be underground)
  //   if (Math.abs(visionPose.getZ()) > MAX_Z_HEIGHT_M) {
  //     return RejectionReason.Z_HEIGHT;
  //   }

  //   // Gate 3: Roll/pitch sanity (extreme tilt = bad PnP solve)
  //   double rollDeg = Math.toDegrees(visionPose.getRotation().getX());
  //   double pitchDeg = Math.toDegrees(visionPose.getRotation().getY());
  //   if (Math.abs(rollDeg) > MAX_ROLL_PITCH_DEG || Math.abs(pitchDeg) > MAX_ROLL_PITCH_DEG) {
  //     return RejectionReason.ROLL_PITCH;
  //   }

  //   // Gate 4: Field bounds (robot can't be outside the walls)
  //   Pose2d pose2d = visionPose.toPose2d();
  //   double x = pose2d.getX();
  //   double y = pose2d.getY();
  //   if (x < -FIELD_MARGIN_M
  //       || x > FIELD_LENGTH_M + FIELD_MARGIN_M
  //       || y < -FIELD_MARGIN_M
  //       || y > FIELD_WIDTH_M + FIELD_MARGIN_M) {
  //     return RejectionReason.FIELD_BOUNDS;
  //   }

  //   // Gate 5: Heading divergence (single-tag heading is unreliable, gyro is truth)
  //   if (tagCount == 1 && gyroHeading != null) {
  //     double headingDiffDeg = Math.abs(pose2d.getRotation().minus(gyroHeading).getDegrees());
  //     if (headingDiffDeg > MAX_HEADING_DIVERGENCE_DEG) {
  //       return RejectionReason.HEADING_DIVERGENCE;
  //     }
  //   }

  //   // Gate 6: Pose jump (can't teleport, but skip during early auto).
  //   // Under defense the robot can genuinely be shoved several meters.
  //   // if (currentPose != null && autoElapsedSec > AUTO_GRACE_PERIOD_SEC) {
  //   //   double jumpM = pose2d.getTranslation().getDistance(currentPose.getTranslation());
  //   //   double jumpLimit = underDefense ? MAX_POSE_JUMP_M * 2.0 : MAX_POSE_JUMP_M;
  //   //   if (jumpM > jumpLimit) {
  //   //     return RejectionReason.POSE_JUMP;
  //   //   }
  //   // }

  //   // Gate 7: Gyro rate. Skip under defense because wheel slip makes odometry
  //   // drift worse than any motion blur on the tags.
  //   if (!underDefense && Math.abs(gyroRateDegPerSec) > MAX_GYRO_RATE_DEG_PER_SEC) {
  //     return RejectionReason.GYRO_RATE;
  //   }

  //   // Gate 8: Opposing alliance single-tag (cross-field single-tag is too noisy)
  //   if (tagCount == 1 && tagIds.length > 0) {
  //     int id = tagIds[0];
  //     boolean tagIsBlue = id <= BLUE_TAG_MAX;
  //     if (tagIsBlue != isBlueAlliance) {
  //       return RejectionReason.OPPOSING_ALLIANCE;
  //     }
  //   }

  //   // Gate 9: Staleness (speed-dependent: tighter when moving fast)
  //   if (ageSec > 0) {
  //     double speedFraction = Math.min(1.0, robotSpeedMps / STALE_SPEED_CUTOFF_MPS);
  //     double staleLimit =
  //         STALE_THRESHOLD_STOPPED_SEC
  //             + (STALE_THRESHOLD_FAST_SEC - STALE_THRESHOLD_STOPPED_SEC) * speedFraction;
  //     if (ageSec > staleLimit) {
  //       return RejectionReason.STALE;
  //     }
  //   }

  //   // Gate 10: Distance (single-tag beyond 5m has too much pixel error)
  //   if (tagCount == 1 && avgTagDistanceM > MAX_SINGLE_TAG_DISTANCE_M) {
  //     return RejectionReason.DISTANCE;
  //   }

    return RejectionReason.ACCEPTED;
  // }
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

  /** High gyro rate + low wheel speed = opponent is spinning us, not us driving fast. */
  // public static boolean isUnderDefense(double gyroRateDps, double speedMps) {
  //   return Math.abs(gyroRateDps) > DEFENSE_GYRO_RATE_DPS && speedMps < DEFENSE_MAX_SPEED_MPS;
  // }

  private VisionFilter() {}
}

