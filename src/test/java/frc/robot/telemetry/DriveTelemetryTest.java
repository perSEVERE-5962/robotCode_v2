package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveTelemetryTest extends TelemetryTestBase {

  private DriveTelemetry telemetry;

  @BeforeEach
  void setUp() {
    telemetry = new DriveTelemetry();
  }

  @Test
  void testDefaultPoseWhenNull() throws Exception {
    telemetry.update();
    Pose2d pose = getField(telemetry, "pose");
    assertNotNull(pose);
    assertEquals(0, pose.getX(), 0.01);
    assertEquals(0, pose.getY(), 0.01);
  }

  @Test
  void testModuleStatesInitialized() throws Exception {
    SwerveModuleState[] states = getField(telemetry, "moduleStates");
    assertNotNull(states);
    assertEquals(4, states.length, "Should have 4 module states");
  }

  @Test
  void testSetTargetPoseEnablesFollowing() {
    assertFalse(telemetry.isFollowingPath());

    telemetry.setTargetPose(new Pose2d(3, 4, Rotation2d.fromDegrees(90)));
    assertTrue(telemetry.isFollowingPath(), "Should be following after setTargetPose");
  }

  @Test
  void testPathCompleteResetsState() {
    telemetry.setTargetPose(new Pose2d(1, 2, new Rotation2d()));
    assertTrue(telemetry.isFollowingPath());

    telemetry.pathComplete();
    assertFalse(telemetry.isFollowingPath(), "Should stop following after pathComplete");
    assertEquals(0, telemetry.getPositionErrorMeters(), 0.01);
    assertEquals(0, telemetry.getHeadingErrorDegrees(), 0.01);
  }

  @Test
  void testSetTargetPoseNullSafety() throws Exception {
    assertDoesNotThrow(() -> telemetry.setTargetPose(null));
    Pose2d target = getField(telemetry, "targetPose");
    assertNotNull(target, "Null target should be replaced with default Pose2d");
  }

  @Test
  void testSetPathNameNullSafety() throws Exception {
    assertDoesNotThrow(() -> telemetry.setPathName(null));
    String name = getField(telemetry, "currentPathName");
    assertEquals("none", name, "Null path name should default to 'none'");
  }

  @Test
  void testSwerveHealthDefaultsDisconnected() {
    telemetry.update();
    for (int i = 0; i < 4; i++) {
      assertFalse(
          telemetry.isDriveMotorConnected(i), "Drive motor " + i + " should default disconnected");
      assertFalse(
          telemetry.isTurnMotorConnected(i), "Turn motor " + i + " should default disconnected");
      assertFalse(telemetry.isEncoderOk(i), "Encoder " + i + " should default to issue");
    }
  }

  @Test
  void testSwerveHealthAccessorBoundsCheck() {
    assertFalse(telemetry.isDriveMotorConnected(-1), "Negative index should return false");
    assertFalse(telemetry.isDriveMotorConnected(4), "Out of bounds index should return false");
    assertFalse(telemetry.isTurnMotorConnected(-1));
    assertFalse(telemetry.isTurnMotorConnected(4));
    assertFalse(telemetry.isEncoderOk(-1));
    assertFalse(telemetry.isEncoderOk(4));
    assertEquals(0, telemetry.getDriveFaultsRaw(-1));
    assertEquals(0, telemetry.getDriveFaultsRaw(4));
    assertEquals(0, telemetry.getTurnFaultsRaw(-1));
    assertEquals(0, telemetry.getTurnFaultsRaw(4));
  }
}
