package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SystemHealthTelemetryTest extends TelemetryTestBase {

  private SystemHealthTelemetry telemetry;

  @BeforeEach
  void setUp() {
    telemetry = new SystemHealthTelemetry();
  }

  @Test
  void testLoopTimingTracked() {
    telemetry.update();
    advanceTime();
    telemetry.update();

    assertTrue(
        telemetry.getLoopTimeMs() > 0, "Loop time should be positive after two updates with delay");
  }

  @Test
  void testLoopOverrunDetection() throws InterruptedException {
    telemetry.update();
    // Sleep longer than LOOP_OVERRUN_THRESHOLD_MS (25ms)
    Thread.sleep(40);
    telemetry.update();

    assertTrue(
        telemetry.getLoopOverrunCount() > 0,
        "Overrun count should increment when loop exceeds 25ms");
  }

  @Test
  void testBrownoutRiskLevelZero() {
    RoboRioSim.setVInVoltage(12.5);

    telemetry.update();
    advanceTime();
    telemetry.update();

    assertEquals(0, telemetry.getBrownoutRiskLevel(), "Brownout risk should be 0 at 12.5V");
    assertFalse(telemetry.isBrownoutRisk(), "Brownout risk flag should be false at 12.5V");
  }

  @Test
  void testBatteryVoltageRead() {
    RoboRioSim.setVInVoltage(11.8);

    assertDoesNotThrow(
        () -> {
          telemetry.update();
          telemetry.log();
        });
  }

  @Test
  void testCurrentDrawDefaults() {
    // Before any update, current draw should be zero
    assertEquals(0.0, telemetry.getTotalCurrentAmps(), 0.001, "Initial total current should be 0");
    assertEquals(0.0, telemetry.getPeakCurrentAmps(), 0.001, "Initial peak current should be 0");
  }

  @Test
  void testCurrentDrawAfterUpdate() {
    // PDH is null in test env, so current stays at 0
    telemetry.update();

    assertEquals(
        0.0, telemetry.getTotalCurrentAmps(), 0.001, "Total current should be 0 without PDH");
    assertEquals(
        0.0, telemetry.getPeakCurrentAmps(), 0.001, "Peak current should be 0 without PDH");
  }

  @Test
  void testGetNameReturnsSystemHealth() {
    assertEquals("SystemHealth", telemetry.getName());
  }

  @Test
  void testVoltageSlopeInitiallyZero() {
    // First update has no prior reading, slope should be 0
    telemetry.update();
    assertEquals(
        0.0, telemetry.getVoltageSlope(), 0.001, "Voltage slope should be 0 on first update");
  }

  @Test
  void testUpdateAndLogDoNotThrow() {
    assertDoesNotThrow(
        () -> {
          telemetry.update();
          telemetry.log();
        });
  }

  @Test
  void testMultipleUpdatesDoNotThrow() {
    assertDoesNotThrow(
        () -> {
          for (int i = 0; i < 20; i++) {
            telemetry.update();
            telemetry.log();
          }
        });
  }
}
