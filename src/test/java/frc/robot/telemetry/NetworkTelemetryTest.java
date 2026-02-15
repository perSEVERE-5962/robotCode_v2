package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class NetworkTelemetryTest extends TelemetryTestBase {

  private NetworkTelemetry telemetry;

  @BeforeEach
  void setUp() {
    telemetry = new NetworkTelemetry();
  }

  @Test
  void testInitialBandwidthZero() {
    assertEquals(
        0.0, telemetry.getBandwidthPercent(), 0.001, "Bandwidth should be 0 before any update");
  }

  @Test
  void testBandwidthEstimationDSNotAttached() {
    DriverStationSim.setDsAttached(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    // Run multiple updates so smoothing converges toward 0
    for (int i = 0; i < 50; i++) {
      telemetry.update();
    }

    // With DS detached, estimated bandwidth is 0; smoothing approaches 0
    // Use generous bound since smoothing may not fully converge
    double percent = telemetry.getBandwidthPercent();
    assertTrue(percent < 10.0, "Bandwidth should be low when DS is not attached, was: " + percent);
    assertFalse(telemetry.isWarning());
    assertFalse(telemetry.isCritical());
  }

  @Test
  void testBandwidthEstimationDSAttachedDisabled() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    telemetry.update();

    // DS attached but disabled: base traffic only (~0.4 Mbps / 7.0 Mbps = ~5.7%)
    double percent = telemetry.getBandwidthPercent();
    assertTrue(percent > 0, "Bandwidth should be > 0 when DS attached");
    assertTrue(percent < 20, "Bandwidth should be low when disabled (no camera)");
  }

  @Test
  void testBandwidthEstimationDSAttachedEnabled() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();

    telemetry.update();

    // Enabled: base + camera (~1.9 Mbps / 7.0 Mbps = ~27%)
    double percent = telemetry.getBandwidthPercent();
    assertTrue(percent > 10, "Bandwidth should include camera estimate when enabled");
    assertTrue(percent < 50, "Bandwidth should still be well under warning threshold");
  }

  @Test
  void testBandwidthWarningThresholds() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();

    telemetry.update();

    // At ~27% bandwidth, should not be warning or critical
    assertFalse(telemetry.isWarning(), "Should not be warning at typical bandwidth levels");
    assertFalse(telemetry.isCritical(), "Should not be critical at typical bandwidth levels");
  }

  @Test
  void testWarningAndCriticalConsistency() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    telemetry.update();

    // Critical implies warning (critical threshold > warning threshold)
    if (telemetry.isCritical()) {
      assertTrue(telemetry.isWarning(), "Critical should always imply warning");
    }
  }

  @Test
  void testSmoothingOverMultipleUpdates() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    for (int i = 0; i < 10; i++) {
      telemetry.update();
    }

    double percent = telemetry.getBandwidthPercent();
    assertTrue(percent > 0, "Bandwidth should stabilize above 0 after multiple updates");
  }

  @Test
  void testGetNameReturnsNetwork() {
    assertEquals("Network", telemetry.getName());
  }

  @Test
  void testUpdateAndLogDoNotThrow() {
    assertDoesNotThrow(
        () -> {
          telemetry.update();
          telemetry.log();
        });
  }
}
