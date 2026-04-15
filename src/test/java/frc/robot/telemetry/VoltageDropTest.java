package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.lang.reflect.Field;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for the shared voltage drop computation in SystemHealthTelemetry. Uses reflection to
 * seed the shared battery voltage snapshot, since the real path needs a running RoboRIO and these
 * tests need to run in plain JUnit.
 */
class VoltageDropTest {

  @BeforeEach
  void resetSharedBatteryVoltage() throws Exception {
    writeSharedBatteryVoltage(Double.NaN);
  }

  @Test
  void healthyDropReadsCorrectly() throws Exception {
    writeSharedBatteryVoltage(12.0);
    double drop = SystemHealthTelemetry.computeVoltageDrop(11.5);
    assertEquals(0.5, drop, 1e-9);
  }

  @Test
  void noiseCaseWithBusAboveBatteryClampsToZero() throws Exception {
    writeSharedBatteryVoltage(11.0);
    double drop = SystemHealthTelemetry.computeVoltageDrop(11.2);
    assertEquals(0.0, drop, 0.0);
  }

  @Test
  void nanBatterySnapshotReturnsZero() throws Exception {
    // battery snapshot is NaN (no update cycle has run yet)
    double drop = SystemHealthTelemetry.computeVoltageDrop(11.5);
    assertEquals(0.0, drop, 0.0);
  }

  @Test
  void nanBusVoltageReturnsZero() throws Exception {
    writeSharedBatteryVoltage(12.0);
    double drop = SystemHealthTelemetry.computeVoltageDrop(Double.NaN);
    assertEquals(0.0, drop, 0.0);
  }

  @Test
  void badCrimpCaseReadsHigh() throws Exception {
    writeSharedBatteryVoltage(12.2);
    double drop = SystemHealthTelemetry.computeVoltageDrop(10.5);
    assertEquals(1.7, drop, 1e-9);
  }

  private static void writeSharedBatteryVoltage(double value) throws Exception {
    Field field = SystemHealthTelemetry.class.getDeclaredField("sharedBatteryVoltage");
    field.setAccessible(true);
    field.setDouble(null, value);
  }
}
