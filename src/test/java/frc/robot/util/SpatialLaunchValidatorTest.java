package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.SpatialLaunchConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SpatialLaunchValidatorTest {

  @BeforeEach
  void setUp() {
    HAL.initialize(500, 0);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
  }

  @Test
  void openFieldIsValid() {
    SpatialLaunchValidator.ValidationResult r = SpatialLaunchValidator.validate(poseAt(2.0, 4.0));
    assertTrue(r.isValid());
    assertTrue(r.pathClear);
    assertTrue(r.distanceValid);
  }

  @Test
  void tooCloseToHubIsInvalid() {
    SpatialLaunchValidator.ValidationResult r = SpatialLaunchValidator.validate(poseAt(4.1, 4.0));
    assertFalse(r.distanceValid);
    assertFalse(r.isValid());
  }

  @Test
  void exactlyAtMinDistanceIsValid() {
    double hubX = 4.5974;
    double hubY = 4.035;
    double minDist = SpatialLaunchConstants.MIN_LAUNCH_DISTANCE_M;
    SpatialLaunchValidator.ValidationResult r =
        SpatialLaunchValidator.validate(poseAt(hubX - minDist, hubY));
    assertTrue(r.distanceValid);
  }

  @Test
  void trenchPillarBlocksPath() {
    // blue near-side pillar is at X [3.96, 4.265], Y [1.265, 1.57]
    // ray from (3.5, 0.0) to blue hub passes through the pillar
    SpatialLaunchValidator.ValidationResult r = SpatialLaunchValidator.validate(poseAt(3.5, 0.0));
    assertFalse(r.pathClear);
    assertFalse(r.isValid());
  }

  @Test
  void missedPillarIsValid() {
    SpatialLaunchValidator.ValidationResult r = SpatialLaunchValidator.validate(poseAt(2.5, 3.0));
    assertTrue(r.pathClear);
    assertTrue(r.isValid());
  }

  @Test
  void redAlliancePillarBlocks() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();

    SpatialLaunchValidator.ValidationResult r = SpatialLaunchValidator.validate(poseAt(13.0, 0.0));
    assertFalse(r.pathClear);
  }

  @Test
  void parallelRayMissesCorrectly() {
    assertTrue(
        SpatialLaunchValidator.isPathClear(
            new Translation2d(3.5, 4.0), new Translation2d(4.5974, 4.0)));
  }

  @Test
  void segmentIntersectsAABBBasic() {
    assertTrue(SpatialLaunchValidator.segmentIntersectsAABB(0, 0.5, 2, 0.5, 0.5, 0, 1.5, 1));
    assertFalse(SpatialLaunchValidator.segmentIntersectsAABB(0, 2, 2, 2, 0.5, 0, 1.5, 1));
  }

  @Test
  void distanceFieldIsPopulated() {
    SpatialLaunchValidator.ValidationResult r = SpatialLaunchValidator.validate(poseAt(2.0, 4.0));
    assertTrue(r.distanceToHub > 0);
    assertEquals(2.6256, r.distanceToHub, 0.01);
  }

  @Test
  void staticExclusionUnderTrench() {
    assertTrue(SpatialLaunchValidator.isInExclusionZone(4.5, 2.5));
  }

  @Test
  void staticExclusionOpenField() {
    assertFalse(SpatialLaunchValidator.isInExclusionZone(8.0, 4.0));
  }

  @Test
  void velocityBufferGrowsWithSpeed() {
    double buffer = SpatialLaunchValidator.computeTrenchBuffer(4.0);
    assertTrue(buffer > 0.5, "buffer should be at least 0.5m at 4 m/s");
    assertEquals(1.0, buffer, 0.01);
  }

  @Test
  void velocityBufferZeroWhenStationary() {
    assertEquals(0.0, SpatialLaunchValidator.computeTrenchBuffer(0.0));
  }

  @Test
  void velocityBufferZeroWhenMovingAway() {
    assertEquals(0.0, SpatialLaunchValidator.computeTrenchBuffer(-2.0));
  }

  @Test
  void expandedZoneDetectsApproach() {
    assertTrue(SpatialLaunchValidator.isInExpandedExclusionZone(3.5, 2.5, 2.0, 0.0));
  }

  @Test
  void expandedZoneIgnoresSlowRobot() {
    assertFalse(SpatialLaunchValidator.isInExpandedExclusionZone(3.5, 2.5, 0.05, 0.0));
  }

  @Test
  void nearestTrenchDistanceInsideIsZero() {
    assertEquals(0.0, SpatialLaunchValidator.getNearestTrenchDistM(4.5, 2.5), 0.01);
  }

  @Test
  void nearestTrenchDistanceOutsideIsPositive() {
    assertTrue(SpatialLaunchValidator.getNearestTrenchDistM(8.0, 4.0) > 1.0);
  }

  @Test
  void validationIncludesExclusionZone() {
    SpatialLaunchValidator.ValidationResult r = SpatialLaunchValidator.validate(poseAt(4.5, 2.5));
    assertFalse(r.outsideExclusionZone);
    assertFalse(r.isValid());
  }

  private static Pose2d poseAt(double x, double y) {
    return new Pose2d(x, y, new Rotation2d());
  }
}
