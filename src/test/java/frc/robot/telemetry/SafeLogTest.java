package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SafeLogTest {

  @BeforeEach
  void setUp() {
    HAL.initialize(500, 0);
    SafeLog.logAndReset();
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
  void testPutNullStringValueDoesNotCrash() {
    assertDoesNotThrow(() -> SafeLog.put("Test/NullStr", (String) null));
  }

  @Test
  void testPutNullPose2dDoesNotCrash() {
    assertDoesNotThrow(() -> SafeLog.put("Test/NullPose", (Pose2d) null));
  }

  @Test
  void testPutSpecialDoubleValues() {
    assertDoesNotThrow(() -> SafeLog.put("Test/NaN", Double.NaN));
    assertDoesNotThrow(() -> SafeLog.put("Test/PosInf", Double.POSITIVE_INFINITY));
    assertDoesNotThrow(() -> SafeLog.put("Test/NegInf", Double.NEGATIVE_INFINITY));
    assertDoesNotThrow(() -> SafeLog.put("Test/MaxVal", Double.MAX_VALUE));
  }
}
