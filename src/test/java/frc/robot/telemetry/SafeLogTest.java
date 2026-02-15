package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SafeLogTest {

  @BeforeEach
  void setUp() {
    HAL.initialize(500, 0);
    SafeLog.logAndReset();
  }

  @Test
  void testInitialFailureCountIsZero() {
    assertEquals(0, SafeLog.getCycleFailures());
  }

  @Test
  void testInitialLastFailedKeyIsEmpty() {
    assertEquals("", SafeLog.getLastFailedKey());
  }

  @Test
  void testPutDouble() {
    assertDoesNotThrow(() -> SafeLog.put("Test/Double", 3.14));
  }

  @Test
  void testPutBoolean() {
    assertDoesNotThrow(() -> SafeLog.put("Test/Bool", true));
  }

  @Test
  void testPutInt() {
    assertDoesNotThrow(() -> SafeLog.put("Test/Int", 42));
  }

  @Test
  void testPutLong() {
    assertDoesNotThrow(() -> SafeLog.put("Test/Long", 123456789L));
  }

  @Test
  void testPutString() {
    assertDoesNotThrow(() -> SafeLog.put("Test/String", "hello"));
  }

  @Test
  void testPutDoubleArray() {
    assertDoesNotThrow(() -> SafeLog.put("Test/DoubleArr", new double[] {1.0, 2.0}));
  }

  @Test
  void testPutStringArray() {
    assertDoesNotThrow(() -> SafeLog.put("Test/StringArr", new String[] {"a", "b"}));
  }

  @Test
  void testPutIntArray() {
    assertDoesNotThrow(() -> SafeLog.put("Test/IntArr", new int[] {1, 2, 3}));
  }

  @Test
  void testPutPose2d() {
    assertDoesNotThrow(() -> SafeLog.put("Test/Pose2d", new Pose2d()));
  }

  @Test
  void testPutPose3d() {
    assertDoesNotThrow(() -> SafeLog.put("Test/Pose3d", new Pose3d()));
  }

  @Test
  void testPutPose3dArray() {
    assertDoesNotThrow(() -> SafeLog.put("Test/Pose3dArr", new Pose3d[] {new Pose3d()}));
  }

  @Test
  void testPutSwerveModuleStates() {
    SwerveModuleState[] states = {new SwerveModuleState(0, Rotation2d.kZero)};
    assertDoesNotThrow(() -> SafeLog.put("Test/Swerve", states));
  }

  @Test
  void testRunExecutesNormally() {
    boolean[] ran = {false};
    SafeLog.run(() -> ran[0] = true);
    assertTrue(ran[0]);
  }

  @Test
  void testRunSwallowsRuntimeException() {
    assertDoesNotThrow(
        () ->
            SafeLog.run(
                () -> {
                  throw new RuntimeException("boom");
                }));
  }

  @Test
  void testRunSwallowsError() {
    assertDoesNotThrow(
        () ->
            SafeLog.run(
                () -> {
                  throw new StackOverflowError("simulated");
                }));
  }

  @Test
  void testRunHandlesNullRunnable() {
    assertDoesNotThrow(() -> SafeLog.run(null));
  }

  @Test
  void testMultipleRunFailuresIndependent() {
    SafeLog.run(
        () -> {
          throw new RuntimeException("first");
        });
    boolean[] secondRan = {false};
    SafeLog.run(() -> secondRan[0] = true);
    assertTrue(secondRan[0], "Second run should execute despite first failure");
  }

  @Test
  void testLogAndResetClearsFailures() {
    SafeLog.logAndReset();
    assertEquals(0, SafeLog.getCycleFailures());
    assertEquals("", SafeLog.getLastFailedKey());
  }

  @Test
  void testLogAndResetDoesNotThrow() {
    assertDoesNotThrow(() -> SafeLog.logAndReset());
  }

  @Test
  void testLogAndResetIdempotent() {
    SafeLog.logAndReset();
    SafeLog.logAndReset();
    assertEquals(0, SafeLog.getCycleFailures());
  }

  @Test
  void testPutNullStringValueDoesNotCrash() {
    assertDoesNotThrow(() -> SafeLog.put("Test/NullStr", (String) null));
  }

  @Test
  void testPutEmptyKey() {
    assertDoesNotThrow(() -> SafeLog.put("", 1.0));
  }

  @Test
  void testPutEmptyArrays() {
    assertDoesNotThrow(() -> SafeLog.put("Test/EmptyDouble", new double[] {}));
    assertDoesNotThrow(() -> SafeLog.put("Test/EmptyString", new String[] {}));
    assertDoesNotThrow(() -> SafeLog.put("Test/EmptyInt", new int[] {}));
  }

  @Test
  void testPutNullPose2dDoesNotCrash() {
    assertDoesNotThrow(() -> SafeLog.put("Test/NullPose", (Pose2d) null));
  }

  @Test
  void testPutNullPose3dArrayDoesNotCrash() {
    assertDoesNotThrow(() -> SafeLog.put("Test/NullPose3dArr", (Pose3d[]) null));
  }

  @Test
  void testPutNullSwerveStatesDoesNotCrash() {
    assertDoesNotThrow(() -> SafeLog.put("Test/NullSwerve", (SwerveModuleState[]) null));
  }

  @Test
  void testPutSpecialDoubleValues() {
    assertDoesNotThrow(() -> SafeLog.put("Test/NaN", Double.NaN));
    assertDoesNotThrow(() -> SafeLog.put("Test/PosInf", Double.POSITIVE_INFINITY));
    assertDoesNotThrow(() -> SafeLog.put("Test/NegInf", Double.NEGATIVE_INFINITY));
    assertDoesNotThrow(() -> SafeLog.put("Test/MaxVal", Double.MAX_VALUE));
  }

  @Test
  void testPutMaxIntValues() {
    assertDoesNotThrow(() -> SafeLog.put("Test/MaxInt", Integer.MAX_VALUE));
    assertDoesNotThrow(() -> SafeLog.put("Test/MinInt", Integer.MIN_VALUE));
  }
}
