package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Method;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class VisionTelemetryTest extends TelemetryTestBase {

  private VisionTelemetry telemetry;

  @BeforeEach
  void setUp() {
    telemetry = new VisionTelemetry();
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
    double conf = invokeComputeConfidence(1.0, 15, true);
    assertEquals(100, conf, 0.01);
  }

  @Test
  void testConfidencePenalizedAtFarDistance() throws Exception {
    double conf = invokeComputeConfidence(6.0, 15, false);
    assertEquals(70, conf, 0.01);
  }

  @Test
  void testConfidenceWorstCase() throws Exception {
    double conf = invokeComputeConfidence(6.0, 1, false);
    assertEquals(40, conf, 0.01);
  }

  @Test
  void testConfidenceMidRange() throws Exception {
    double conf = invokeComputeConfidence(3.5, 7, false);
    assertEquals(80, conf, 0.01);
  }

  @Test
  void testConfidenceZeroWhenNoTarget() throws Exception {
    double conf = invokeComputeConfidence(0, 0, false);
    assertEquals(0, conf, 0.01);
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
    assertEquals(0.05, stdDevs[0], 0.001);
    assertEquals(0.05, stdDevs[1], 0.001);
    assertEquals(0.025, stdDevs[2], 0.001);
  }

  @Test
  void testStdDevsFarDistance() throws Exception {
    invokeUpdateStdDevs(5.0);
    double[] stdDevs = getField(telemetry, "measurementStdDevs");
    assertEquals(0.25, stdDevs[0], 0.001);
    assertEquals(0.25, stdDevs[1], 0.001);
    assertEquals(0.125, stdDevs[2], 0.001);
  }

  @Test
  void testStdDevsCappedAtMax() throws Exception {
    invokeUpdateStdDevs(10.0);
    double[] stdDevs = getField(telemetry, "measurementStdDevs");
    assertEquals(1.0, stdDevs[0], 0.001);
    assertEquals(1.0, stdDevs[1], 0.001);
    assertEquals(0.5, stdDevs[2], 0.001);
  }

  @Test
  void testSanitizeNaNReturnsZero() throws Exception {
    Method m = VisionTelemetry.class.getDeclaredMethod("sanitize", double.class);
    m.setAccessible(true);
    assertEquals(0.0, (double) m.invoke(telemetry, Double.NaN), 0.01);
  }
}
