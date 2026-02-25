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

    assertEquals(180.0, result.getRotation().getDegrees(), 1.0);
  }

  @Test
  void testRobotWithinArcNotClamped() {
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
    Pose2d result =
        HubScoringUtil.getClosestScoringPose(
            new Translation2d(0, 5), HUB, SCORING_DIST, Rotation2d.fromDegrees(0), 90);

    double cos45 = SCORING_DIST * Math.cos(Math.toRadians(45));
    double sin45 = SCORING_DIST * Math.sin(Math.toRadians(45));
    assertEquals(cos45, result.getX(), DELTA);
    assertEquals(sin45, result.getY(), DELTA);
  }

  @Test
  void testFullCircleArcProjectsToNearestPoint() {
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
  void testFacingAlwaysTowardHub() {
    double[] testAngles = {0, 30, 45, 90, 135, -45, -90, -135};

    for (double deg : testAngles) {
      double rad = Math.toRadians(deg);
      Translation2d robot = new Translation2d(5 * Math.cos(rad), 5 * Math.sin(rad));

      Pose2d result =
          HubScoringUtil.getClosestScoringPose(
              robot, HUB, SCORING_DIST, Rotation2d.fromDegrees(deg), 360);

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
  void testSymmetryAboutScoringCenter() {
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
