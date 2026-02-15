package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class HubScoringUtilTest {

  private static final double DELTA = 0.02;
  private static final Translation2d HUB = new Translation2d(0, 0);
  private static final double SCORING_DIST = 2.0;

  @Test
  void testRobotDirectlyOnScoringLine() {
    // Robot east of hub, arc centered east
    Pose2d result =
        HubScoringUtil.getClosestScoringPose(
            new Translation2d(5, 0), HUB, SCORING_DIST, Rotation2d.fromDegrees(0), 90);

    assertEquals(2.0, result.getX(), DELTA);
    assertEquals(0.0, result.getY(), DELTA);
  }

  @Test
  void testRobotFacesHub() {
    Pose2d result =
        HubScoringUtil.getClosestScoringPose(
            new Translation2d(5, 0), HUB, SCORING_DIST, Rotation2d.fromDegrees(0), 90);

    // At angle 0 on arc, facing = 0 + PI = 180 degrees (toward hub)
    assertEquals(180.0, result.getRotation().getDegrees(), 1.0);
  }

  @Test
  void testRobotWithinArcNotClamped() {
    // Robot at 20 degrees from hub, arc spans -45 to 45
    double angle = Math.toRadians(20);
    Translation2d robot = new Translation2d(5 * Math.cos(angle), 5 * Math.sin(angle));

    Pose2d result =
        HubScoringUtil.getClosestScoringPose(
            robot, HUB, SCORING_DIST, Rotation2d.fromDegrees(0), 90);

    double expectedX = SCORING_DIST * Math.cos(angle);
    double expectedY = SCORING_DIST * Math.sin(angle);
    assertEquals(expectedX, result.getX(), DELTA);
    assertEquals(expectedY, result.getY(), DELTA);
  }

  @Test
  void testRobotOutsideArcClampedToPositiveEdge() {
    // Robot at 90 degrees (north), arc is 0 +/- 45 -> edge at 45
    Pose2d result =
        HubScoringUtil.getClosestScoringPose(
            new Translation2d(0, 5), HUB, SCORING_DIST, Rotation2d.fromDegrees(0), 90);

    double cos45 = SCORING_DIST * Math.cos(Math.toRadians(45));
    double sin45 = SCORING_DIST * Math.sin(Math.toRadians(45));
    assertEquals(cos45, result.getX(), DELTA);
    assertEquals(sin45, result.getY(), DELTA);
  }

  @Test
  void testRobotOutsideArcClampedToNegativeEdge() {
    // Robot at -90 degrees (south), arc is 0 +/- 45 -> edge at -45
    Pose2d result =
        HubScoringUtil.getClosestScoringPose(
            new Translation2d(0, -5), HUB, SCORING_DIST, Rotation2d.fromDegrees(0), 90);

    double cos = SCORING_DIST * Math.cos(Math.toRadians(-45));
    double sin = SCORING_DIST * Math.sin(Math.toRadians(-45));
    assertEquals(cos, result.getX(), DELTA);
    assertEquals(sin, result.getY(), DELTA);
  }

  @Test
  void testScoringDistancePreservedForMultiplePositions() {
    Translation2d[] positions = {
      new Translation2d(3, 4),
      new Translation2d(-2, 1),
      new Translation2d(0.5, -3),
      new Translation2d(10, 0),
      new Translation2d(-5, -5),
    };

    for (Translation2d pos : positions) {
      Pose2d result =
          HubScoringUtil.getClosestScoringPose(
              pos, HUB, SCORING_DIST, Rotation2d.fromDegrees(0), 360);
      double dist = result.getTranslation().getDistance(HUB);
      assertEquals(SCORING_DIST, dist, DELTA, "Scoring distance not preserved for robot at " + pos);
    }
  }

  @Test
  void testFullCircleArcProjectsToNearestPoint() {
    // 360 arc = full circle, projects directly toward robot
    Translation2d robot = new Translation2d(3, 4);
    double angleToRobot = Math.atan2(4, 3);

    Pose2d result =
        HubScoringUtil.getClosestScoringPose(
            robot, HUB, SCORING_DIST, Rotation2d.fromDegrees(0), 360);

    double expectedX = SCORING_DIST * Math.cos(angleToRobot);
    double expectedY = SCORING_DIST * Math.sin(angleToRobot);
    assertEquals(expectedX, result.getX(), DELTA);
    assertEquals(expectedY, result.getY(), DELTA);
  }

  @Test
  void testOffsetHubCenter() {
    Translation2d hub = new Translation2d(8, 4);
    Translation2d robot = new Translation2d(12, 4); // East of offset hub

    Pose2d result =
        HubScoringUtil.getClosestScoringPose(robot, hub, 3.0, Rotation2d.fromDegrees(0), 90);

    // 3m east of hub center at (8,4)
    assertEquals(11.0, result.getX(), DELTA);
    assertEquals(4.0, result.getY(), DELTA);
  }

  @Test
  void testNorthSideScoring() {
    // Arc centered at 90 degrees (north)
    Pose2d result =
        HubScoringUtil.getClosestScoringPose(
            new Translation2d(0, 5), HUB, SCORING_DIST, Rotation2d.fromDegrees(90), 90);

    assertEquals(0.0, result.getX(), DELTA);
    assertEquals(2.0, result.getY(), DELTA);
    // Facing south (toward hub) = 90 + 180 = 270, normalize to [-180,180]
    double facingDeg = MathUtil.inputModulus(result.getRotation().getDegrees(), -180, 180);
    assertEquals(-90.0, facingDeg, 1.0);
  }

  @Test
  void testNarrowArcConstrainsPosition() {
    // 10 degree arc, robot way off to the side (~53 degrees from east)
    Translation2d robot = new Translation2d(3, 4);

    Pose2d result =
        HubScoringUtil.getClosestScoringPose(
            robot, HUB, SCORING_DIST, Rotation2d.fromDegrees(0), 10);

    // Clamped to +5 degree edge
    double clampedAngle = Math.toRadians(5);
    double expectedX = SCORING_DIST * Math.cos(clampedAngle);
    double expectedY = SCORING_DIST * Math.sin(clampedAngle);
    assertEquals(expectedX, result.getX(), DELTA);
    assertEquals(expectedY, result.getY(), DELTA);
  }

  @Test
  void testFacingAlwaysTowardHub() {
    double[] testAngles = {0, 30, 45, 90, 135, -45, -90, -135};

    for (double deg : testAngles) {
      double rad = Math.toRadians(deg);
      Translation2d robot = new Translation2d(5 * Math.cos(rad), 5 * Math.sin(rad));

      Pose2d result =
          HubScoringUtil.getClosestScoringPose(
              robot, HUB, SCORING_DIST, Rotation2d.fromDegrees(deg), 360);

      // Facing direction: from result position toward hub
      // Normalize both to [-PI, PI] since Rotation2d doesn't auto-normalize
      Translation2d resultToHub = HUB.minus(result.getTranslation());
      double expectedFacing = Math.atan2(resultToHub.getY(), resultToHub.getX());
      double actualFacing = MathUtil.angleModulus(result.getRotation().getRadians());
      double normalizedExpected = MathUtil.angleModulus(expectedFacing);
      assertEquals(
          normalizedExpected,
          actualFacing,
          0.05,
          "Facing incorrect for robot at " + deg + " degrees");
    }
  }

  @Test
  void testDifferentScoringDistances() {
    double[] distances = {1.0, 2.5, 4.0, 6.0};

    for (double dist : distances) {
      Pose2d result =
          HubScoringUtil.getClosestScoringPose(
              new Translation2d(10, 0), HUB, dist, Rotation2d.fromDegrees(0), 90);

      assertEquals(
          dist,
          result.getTranslation().getDistance(HUB),
          DELTA,
          "Distance not preserved for scoringDistance=" + dist);
    }
  }

  @Test
  void testSymmetryAboutScoringCenter() {
    // Robot at +30 and -30 should give symmetric results
    double rad30 = Math.toRadians(30);
    Pose2d above =
        HubScoringUtil.getClosestScoringPose(
            new Translation2d(5 * Math.cos(rad30), 5 * Math.sin(rad30)),
            HUB,
            SCORING_DIST,
            Rotation2d.fromDegrees(0),
            90);

    Pose2d below =
        HubScoringUtil.getClosestScoringPose(
            new Translation2d(5 * Math.cos(-rad30), 5 * Math.sin(-rad30)),
            HUB,
            SCORING_DIST,
            Rotation2d.fromDegrees(0),
            90);

    assertEquals(above.getX(), below.getX(), DELTA, "X should be symmetric");
    assertEquals(above.getY(), -below.getY(), DELTA, "Y should be mirrored");
  }
}
