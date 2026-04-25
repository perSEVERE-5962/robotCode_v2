package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.telemetry.TelemetryManager;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PredictiveAlertsTest {

  private static TelemetryManager tm;
  private PredictiveAlerts alerts;

  @BeforeAll
  static void initAll() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    tm = TelemetryManager.getInstance();
  }

  @BeforeEach
  void setUp() {
    resetSingleton("frc.robot.util.PredictiveAlerts");
    RoboRioSim.setVInVoltage(12.5);
    DriverStationSim.notifyNewData();
    alerts = PredictiveAlerts.getInstance();
    SafeLog.logAndReset();
  }

  @AfterAll
  static void tearDownAll() {
    closeSubsystemMotor("frc.robot.subsystems.Shooter");
    closeSubsystemMotor("frc.robot.subsystems.Indexer");
    closeSubsystemMotor("frc.robot.subsystems.IntakeRoller");
    resetSingleton("frc.robot.util.PredictiveAlerts");
    resetSingleton("frc.robot.telemetry.TelemetryManager");

    resetSingleton("frc.robot.subsystems.Shooter");
    resetSingleton("frc.robot.subsystems.Indexer");
    resetSingleton("frc.robot.subsystems.IntakeRoller");
  }

  @Test
  void testNoPredictionsBeforeBufferFull() {
    alerts.update();
    assertFalse(alerts.isBatteryAtRisk());
    assertFalse(alerts.isTempAtRisk());
  }

  @Test
  void testFlatVoltageSlopeIsZero() throws Exception {
    double[] samples = getField(alerts, "voltageSamples");
    for (int i = 0; i < samples.length; i++) {
      samples[i] = 12.0;
    }
    setField(alerts, "samplesInitialized", true);
    setField(alerts, "sampleIndex", 0);

    double slope = invokeCalculateSlope(samples);
    assertEquals(0.0, slope, 0.001, "Constant voltage should have zero slope");
  }

  @Test
  void testDroppingVoltageSlopeIsNegative() throws Exception {
    double[] samples = getField(alerts, "voltageSamples");
    int n = samples.length;
    for (int i = 0; i < n; i++) {
      samples[i] = 13.0 - (1.5 * i / (n - 1));
    }
    setField(alerts, "samplesInitialized", true);
    setField(alerts, "sampleIndex", 0);

    double slope = invokeCalculateSlope(samples);
    assertTrue(slope < 0, "Dropping voltage should produce negative raw slope");
  }

  @Test
  void testBatteryAtRiskWhenPredictionBelowThreshold() throws Exception {
    setField(alerts, "predictedTimeToWarning", 15.0);
    assertTrue(alerts.isBatteryAtRisk());
  }

  @Test
  void testBatteryNotAtRiskAtExactBoundary() throws Exception {
    setField(alerts, "predictedTimeToWarning", 30.0);
    assertFalse(alerts.isBatteryAtRisk());
  }

  @Test
  void testFullCycleWithDroppingVoltage() throws Exception {
    int sampleSize = getSampleSize();

    for (int i = 0; i < sampleSize + 1; i++) {
      double v = 12.5 - (2.0 * i / sampleSize);
      RoboRioSim.setVInVoltage(v);
      DriverStationSim.notifyNewData();
      alerts.update();
    }

    boolean initialized = getField(alerts, "samplesInitialized");
    assertTrue(initialized, "Buffer should be full after SAMPLE_SIZE+1 updates");

    double dropRate = getField(alerts, "voltageDropRate");
    assertTrue(dropRate > 0, "voltageDropRate should be positive when voltage is falling");
  }

  @Test
  void testVShapedRecoveryNoFalseAlarm() throws Exception {
    double[] samples = getField(alerts, "voltageSamples");
    int n = samples.length;
    for (int i = 0; i < n; i++) {
      double t = (double) i / (n - 1);
      samples[i] = 12.5 - 1.5 * (1.0 - Math.abs(2.0 * t - 1.0));
    }
    setField(alerts, "samplesInitialized", true);
    setField(alerts, "sampleIndex", 0);

    double slope = invokeCalculateSlope(samples);
    double voltageDropRate = -slope;
    assertEquals(
        0.0,
        voltageDropRate,
        0.01,
        "Symmetric V-shaped recovery should have zero drop rate, got " + voltageDropRate);
  }

  @Test
  void testCircularBufferWrapAroundCorrectSlope() throws Exception {
    double[] samples = getField(alerts, "voltageSamples");
    int n = samples.length;
    int offset = n / 2;

    for (int i = 0; i < n; i++) {
      int physicalIdx = (offset + i) % n;
      samples[physicalIdx] = 13.0 - (1.5 * i / (n - 1));
    }
    setField(alerts, "sampleIndex", offset);
    setField(alerts, "samplesInitialized", true);

    double wrappedSlope = invokeCalculateSlope(samples);
    assertTrue(wrappedSlope < 0, "Wrapped buffer should detect dropping voltage");

    double[] nonWrapped = getField(alerts, "voltageSamples");
    for (int i = 0; i < n; i++) {
      nonWrapped[i] = 13.0 - (1.5 * i / (n - 1));
    }
    setField(alerts, "sampleIndex", 0);
    double straightSlope = invokeCalculateSlope(nonWrapped);

    assertEquals(
        straightSlope,
        wrappedSlope,
        0.1,
        "Wrapped and non-wrapped slopes should match for same linear trend");
  }

  @Test
  void testAlertFlagRefiresAfterRecovery() throws Exception {
    setField(alerts, "batteryAlertSent", true);

    setField(alerts, "predictedTimeToWarning", -1.0);
    invokeCheckPredictiveAlerts();
    assertFalse((boolean) getField(alerts, "batteryAlertSent"), "Flag should clear on recovery");

    setField(alerts, "predictedTimeToWarning", 10.0);
    invokeCheckPredictiveAlerts();
    assertTrue(
        (boolean) getField(alerts, "batteryAlertSent"),
        "Alert should re-fire after recovery cleared the flag");
  }

  private int getSampleSize() throws Exception {
    double[] samples = getField(alerts, "voltageSamples");
    return samples.length;
  }

  private double invokeCalculateSlope(double[] samples) throws Exception {
    Method m = PredictiveAlerts.class.getDeclaredMethod("calculateSlope", double[].class);
    m.setAccessible(true);
    return (double) m.invoke(alerts, samples);
  }

  private void invokeCheckPredictiveAlerts() throws Exception {
    Method m = PredictiveAlerts.class.getDeclaredMethod("checkPredictiveAlerts");
    m.setAccessible(true);
    m.invoke(alerts);
  }

  @SuppressWarnings("unchecked")
  private <T> T getField(Object obj, String fieldName) throws Exception {
    Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    return (T) f.get(obj);
  }

  private void setField(Object obj, String fieldName, Object value) throws Exception {
    Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    f.set(obj, value);
  }

  private static void resetSingleton(String className) {
    try {
      Class<?> clazz = Class.forName(className);
      Field f = clazz.getDeclaredField("instance");
      f.setAccessible(true);
      f.set(null, null);
    } catch (Exception e) {
    }
  }

  private static void closeSubsystemMotor(String className) {
    try {
      Class<?> clazz = Class.forName(className);
      Field f = clazz.getDeclaredField("instance");
      f.setAccessible(true);
      Object instance = f.get(null);
      if (instance != null) {
        Method getMotor = clazz.getMethod("getMotor");
        Object motor = getMotor.invoke(instance);
        if (motor instanceof AutoCloseable) {
          ((AutoCloseable) motor).close();
        }
        try {
          Field followersField = clazz.getDeclaredField("followers");
          followersField.setAccessible(true);
          Object[] followers = (Object[]) followersField.get(instance);
          if (followers != null) {
            for (Object follower : followers) {
              if (follower instanceof AutoCloseable) {
                ((AutoCloseable) follower).close();
              }
            }
          }
        } catch (NoSuchFieldException e2) {
        }
      }
    } catch (Exception e) {
    }
  }
}
