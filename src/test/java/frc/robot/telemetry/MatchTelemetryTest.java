package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class MatchTelemetryTest extends TelemetryTestBase {

  private MatchTelemetry telemetry;

  @BeforeEach
  void setUp() {
    telemetry = new MatchTelemetry();
  }

  @Test
  void testDisabledMode() throws Exception {
    // WPILib quirk: isTeleop() returns true when disabled (not auto, not test).
    // MatchTelemetry checks isAutonomous/isTeleop/isTest, so disabled shows as TELEOP.
    // The real indicator is the enabled field.
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();

    telemetry.update();

    boolean enabled = getField(telemetry, "enabled");
    assertFalse(enabled, "Should not be enabled when DriverStation is disabled");
  }

  @Test
  void testAutonomousMode() throws Exception {
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();

    telemetry.update();

    String mode = getField(telemetry, "mode");
    boolean enabled = getField(telemetry, "enabled");
    assertEquals("AUTO", mode);
    assertTrue(enabled);
  }

  @Test
  void testTeleopMode() throws Exception {
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();

    telemetry.update();

    String mode = getField(telemetry, "mode");
    assertEquals("TELEOP", mode);
  }

  @Test
  void testTestMode() throws Exception {
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(true);
    DriverStationSim.notifyNewData();

    telemetry.update();

    String mode = getField(telemetry, "mode");
    assertEquals("TEST", mode);
  }

  @Test
  void testAllianceRed() throws Exception {
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();

    telemetry.update();

    String alliance = getField(telemetry, "alliance");
    assertEquals("Red", alliance);
  }

  @Test
  void testAllianceBlue() throws Exception {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue2);
    DriverStationSim.notifyNewData();

    telemetry.update();

    String alliance = getField(telemetry, "alliance");
    assertEquals("Blue", alliance);
  }

  @Test
  void testFMSAttached() throws Exception {
    DriverStationSim.setFmsAttached(true);
    DriverStationSim.notifyNewData();

    telemetry.update();

    boolean fmsAttached = getField(telemetry, "fmsAttached");
    assertTrue(fmsAttached);
  }

  @Test
  void testMatchTime() throws Exception {
    DriverStationSim.setMatchTime(120.5);
    DriverStationSim.notifyNewData();

    telemetry.update();

    double matchTime = getField(telemetry, "matchTime");
    assertEquals(120.5, matchTime, 0.1);
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
  void testGetNameReturnsMatch() {
    assertEquals("Match", telemetry.getName());
  }

  @Test
  void testAllianceDefaultsToString() throws Exception {
    telemetry.update();
    String alliance = getField(telemetry, "alliance");
    assertNotNull(alliance);
    assertFalse(alliance.isEmpty(), "Alliance should not be empty string");
  }
}
