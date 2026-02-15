package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.MethodName.class)
class DriveTelemetryTest extends TelemetryTestBase {

  private DriveTelemetry telemetry;

  @BeforeEach
  void setUp() {
    telemetry = new DriveTelemetry();
  }

  @Test
  void testGetNameReturnsDrive() {
    assertEquals("Drive", telemetry.getName());
  }

  @Test
  void testUpdateDoesNotThrowWithoutSubsystem() {
    assertDoesNotThrow(
        () -> {
          telemetry.update();
          telemetry.log();
        });
  }

  @Test
  void testLogDoesNotThrowWithoutSubsystem() {
    telemetry.update();
    assertDoesNotThrow(() -> telemetry.log());
  }

  @Test
  void testSubsystemNotAvailableWhenNull() throws Exception {
    telemetry.update();
    boolean available = getField(telemetry, "subsystemAvailable");
    assertFalse(available, "Should not be available without SwerveSubsystem");
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
  void testDefaultSpeedsWhenNull() throws Exception {
    telemetry.update();
    double vx = getField(telemetry, "vxMetersPerSec");
    double vy = getField(telemetry, "vyMetersPerSec");
    double omega = getField(telemetry, "omegaRadPerSec");
    assertEquals(0, vx, 0.01);
    assertEquals(0, vy, 0.01);
    assertEquals(0, omega, 0.01);
  }

  @Test
  void testGyroNotConnectedWhenNull() throws Exception {
    telemetry.update();
    boolean connected = getField(telemetry, "gyroConnected");
    assertFalse(connected, "Gyro should not be connected without SwerveSubsystem");
  }

  @Test
  void testDefaultGyroValuesWhenNull() throws Exception {
    telemetry.update();
    double yaw = getField(telemetry, "yawRad");
    double pitch = getField(telemetry, "pitchRad");
    double roll = getField(telemetry, "rollRad");
    assertEquals(0, yaw, 0.01);
    assertEquals(0, pitch, 0.01);
    assertEquals(0, roll, 0.01);
  }

  @Test
  void testModuleStatesInitialized() throws Exception {
    SwerveModuleState[] states = getField(telemetry, "moduleStates");
    assertNotNull(states);
    assertEquals(4, states.length, "Should have 4 module states");
    for (SwerveModuleState state : states) {
      assertNotNull(state);
    }
  }

  @Test
  void testEncoderHealthDefaultsOk() throws Exception {
    boolean[] absOk = getField(telemetry, "encoderAbsoluteOk");
    assertEquals(4, absOk.length);
    for (boolean ok : absOk) {
      assertTrue(ok, "Encoder health should default to OK");
    }
  }

  @Test
  void testOdometryTimestampUpdated() throws Exception {
    telemetry.update();
    double ts = getField(telemetry, "odometryTimestampSec");
    assertTrue(ts >= 0, "Odometry timestamp should be set after update");
  }

  @Test
  void testSetTargetPoseEnablesFollowing() {
    assertFalse(telemetry.isFollowingPath());

    telemetry.setTargetPose(new Pose2d(3, 4, Rotation2d.fromDegrees(90)));
    assertTrue(telemetry.isFollowingPath(), "Should be following after setTargetPose");
  }

  @Test
  void testSetPathName() throws Exception {
    telemetry.setPathName("AutoPath1");
    String name = getField(telemetry, "currentPathName");
    assertEquals("AutoPath1", name);
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
  void testDefaultPathName() throws Exception {
    String name = getField(telemetry, "currentPathName");
    assertEquals("none", name, "Default path name should be 'none'");
  }
}
