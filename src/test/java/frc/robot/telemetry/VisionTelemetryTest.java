package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Method;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.MethodName.class)
class VisionTelemetryTest extends TelemetryTestBase {

  private VisionTelemetry telemetry;

  @BeforeEach
  void setUp() {
    telemetry = new VisionTelemetry();
  }

  @Test
  void testGetNameReturnsVision() {
    assertEquals("Vision", telemetry.getName());
  }

  @Test
  void testUpdateDoesNotThrowWithoutVision() {
    assertDoesNotThrow(
        () -> {
          telemetry.update();
          telemetry.log();
        });
  }

  @Test
  void testLogDoesNotThrowWithoutVision() {
    telemetry.update();
    assertDoesNotThrow(() -> telemetry.log());
  }

  @Test
  void testSubsystemNotAvailableWhenNull() throws Exception {
    telemetry.update();
    boolean available = getField(telemetry, "subsystemAvailable");
    assertFalse(available);
  }

  @Test
  void testDefaultHasTargetFalse() {
    telemetry.update();
    assertFalse(telemetry.hasTarget());
  }

  @Test
  void testDefaultLockedOnTargetFalse() {
    telemetry.update();
    assertFalse(telemetry.isLockedOnTarget());
  }

  @Test
  void testDefaultLockedTagID() throws Exception {
    telemetry.update();
    int tagID = getField(telemetry, "lockedTagID");
    assertEquals(-1, tagID, "Locked tag ID should default to -1");
  }

  @Test
  void testDefaultBestCameraNone() throws Exception {
    telemetry.update();
    String cam = getField(telemetry, "bestCameraName");
    assertEquals("NONE", cam);
  }

  @Test
  void testDefaultCamerasNotConnected() throws Exception {
    telemetry.update();
    boolean left = getField(telemetry, "leftCamConnected");
    boolean right = getField(telemetry, "rightCamConnected");
    assertFalse(left);
    assertFalse(right);
  }

  @Test
  void testDefaultMeasurementStdDevs() throws Exception {
    telemetry.update();
    double[] stdDevs = getField(telemetry, "measurementStdDevs");
    assertEquals(3, stdDevs.length);
    assertEquals(0.5, stdDevs[0], 0.01);
    assertEquals(0.5, stdDevs[1], 0.01);
    assertEquals(0.5, stdDevs[2], 0.01);
  }

  @Test
  void testDefaultConfidenceZero() throws Exception {
    telemetry.update();
    double conf = getField(telemetry, "poseConfidence");
    assertEquals(0, conf, 0.01);
  }

  @Test
  void testDefaultLatencyZero() throws Exception {
    telemetry.update();
    double lat = getField(telemetry, "latencyMs");
    assertEquals(0, lat, 0.01);
  }

  private double invokeComputeConfidence(double distance, int frameCount, boolean stable)
      throws Exception {
    Method m =
        VisionTelemetry.class.getDeclaredMethod(
            "computeConfidence", double.class, int.class, boolean.class);
    m.setAccessible(true);
    return (double) m.invoke(telemetry, distance, frameCount, stable);
  }

  @Test
  void testConfidenceMaxAtCloseStable() throws Exception {
    // Close (< 2m), many frames (>= 10), stable → 100 + 10 clamped to 100
    double conf = invokeComputeConfidence(1.0, 15, true);
    assertEquals(100, conf, 0.01);
  }

  @Test
  void testConfidencePenalizedAtFarDistance() throws Exception {
    // > 5m: -30, many frames: no penalty, not stable
    double conf = invokeComputeConfidence(6.0, 15, false);
    assertEquals(70, conf, 0.01);
  }

  @Test
  void testConfidencePenalizedFewFrames() throws Exception {
    // Close: no penalty, < 3 frames: -30, not stable
    double conf = invokeComputeConfidence(1.0, 2, false);
    assertEquals(70, conf, 0.01);
  }

  @Test
  void testConfidenceWorstCase() throws Exception {
    // Far (> 5m): -30, few frames (< 3): -30 → 40
    double conf = invokeComputeConfidence(6.0, 1, false);
    assertEquals(40, conf, 0.01);
  }

  @Test
  void testConfidenceMidRange() throws Exception {
    // 3-5m: -15, 5-9 frames: -5, not stable → 80
    double conf = invokeComputeConfidence(3.5, 7, false);
    assertEquals(80, conf, 0.01);
  }

  @Test
  void testConfidenceStableBonus() throws Exception {
    // Close, many frames, stable → 100 + 10 → clamped 100
    // Mid distance (2-3m): -5, 3-4 frames: -15, stable: +10 → 90
    double conf = invokeComputeConfidence(2.5, 4, true);
    assertEquals(90, conf, 0.01);
  }

  private void invokeUpdateStdDevs(double distanceM) throws Exception {
    Method m = VisionTelemetry.class.getDeclaredMethod("updateMeasurementStdDevs", double.class);
    m.setAccessible(true);
    m.invoke(telemetry, distanceM);
  }

  @Test
  void testStdDevsCloseDistance() throws Exception {
    invokeUpdateStdDevs(1.0);
    double[] stdDevs = getField(telemetry, "measurementStdDevs");
    // scale = max(0.5, 1*1*0.1) = max(0.5, 0.1) = 0.5
    // x,y = min(2.0, 0.1*0.5) = 0.05, theta = min(1.0, 0.05*0.5) = 0.025
    assertEquals(0.05, stdDevs[0], 0.001);
    assertEquals(0.05, stdDevs[1], 0.001);
    assertEquals(0.025, stdDevs[2], 0.001);
  }

  @Test
  void testStdDevsFarDistance() throws Exception {
    invokeUpdateStdDevs(5.0);
    double[] stdDevs = getField(telemetry, "measurementStdDevs");
    // scale = max(0.5, 25*0.1) = 2.5
    // x,y = min(2.0, 0.1*2.5) = 0.25, theta = min(1.0, 0.05*2.5) = 0.125
    assertEquals(0.25, stdDevs[0], 0.001);
    assertEquals(0.25, stdDevs[1], 0.001);
    assertEquals(0.125, stdDevs[2], 0.001);
  }

  @Test
  void testStdDevsCappedAtMax() throws Exception {
    invokeUpdateStdDevs(10.0);
    double[] stdDevs = getField(telemetry, "measurementStdDevs");
    // scale = max(0.5, 100*0.1) = 10.0
    // x,y = min(2.0, 0.1*10) = min(2.0, 1.0) = 1.0, theta = min(1.0, 0.05*10) = 0.5
    assertEquals(1.0, stdDevs[0], 0.001);
    assertEquals(1.0, stdDevs[1], 0.001);
    assertEquals(0.5, stdDevs[2], 0.001);
  }

  private double invokeSanitize(double value) throws Exception {
    Method m = VisionTelemetry.class.getDeclaredMethod("sanitize", double.class);
    m.setAccessible(true);
    return (double) m.invoke(telemetry, value);
  }

  @Test
  void testSanitizeFinitePassthrough() throws Exception {
    assertEquals(5.0, invokeSanitize(5.0), 0.01);
  }

  @Test
  void testSanitizeNaNReturnsZero() throws Exception {
    assertEquals(0.0, invokeSanitize(Double.NaN), 0.01);
  }

  @Test
  void testSanitizeInfinityReturnsZero() throws Exception {
    assertEquals(0.0, invokeSanitize(Double.POSITIVE_INFINITY), 0.01);
    assertEquals(0.0, invokeSanitize(Double.NEGATIVE_INFINITY), 0.01);
  }
}
