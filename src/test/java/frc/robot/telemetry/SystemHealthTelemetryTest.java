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
}
