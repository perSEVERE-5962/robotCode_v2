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
  void testBandwidthEstimationDSNotAttached() {
    DriverStationSim.setDsAttached(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    for (int i = 0; i < 50; i++) {
      telemetry.update();
    }

    double percent = telemetry.getBandwidthPercent();
    assertTrue(percent < 10.0, "Bandwidth should be low when DS is not attached, was: " + percent);
    assertFalse(telemetry.isWarning());
    assertFalse(telemetry.isCritical());
  }

  @Test
  void testBandwidthEstimationDSAttachedEnabled() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();

    telemetry.update();

    double percent = telemetry.getBandwidthPercent();
    assertTrue(percent > 10, "Bandwidth should include camera estimate when enabled");
    assertTrue(percent < 50, "Bandwidth should still be well under warning threshold");
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
}
