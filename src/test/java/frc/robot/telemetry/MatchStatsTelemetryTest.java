package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.MethodName.class)
class MatchStatsTelemetryTest extends TelemetryTestBase {

  private ShooterTelemetry shooterTelemetry;
  private VisionTelemetry visionTelemetry;
  private MatchStatsTelemetry telemetry;

  @BeforeEach
  void setUp() {
    shooterTelemetry = new ShooterTelemetry();
    visionTelemetry = new VisionTelemetry();
    telemetry = new MatchStatsTelemetry(shooterTelemetry, visionTelemetry, null);
  }

  @Test
  void testGetNameReturnsMatchStats() {
    assertEquals("MatchStats", telemetry.getName());
  }

  @Test
  void testUpdateDoesNotThrow() {
    assertDoesNotThrow(
        () -> {
          telemetry.update();
          telemetry.log();
        });
  }

  @Test
  void testLogDoesNotThrow() {
    telemetry.update();
    assertDoesNotThrow(() -> telemetry.log());
  }

  @Test
  void testDefaultShotsZero() throws Exception {
    telemetry.update();
    int auto = getField(telemetry, "autoShots");
    int teleop = getField(telemetry, "teleopShots");
    int endgame = getField(telemetry, "endgameShots");
    assertEquals(0, auto);
    assertEquals(0, teleop);
    assertEquals(0, endgame);
  }

  @Test
  void testDefaultTimeAllocationsZero() throws Exception {
    telemetry.update();
    double shooting = getField(telemetry, "timeShootingMs");
    double intaking = getField(telemetry, "timeIntakingMs");
    double aiming = getField(telemetry, "timeAimingMs");
    double idle = getField(telemetry, "timeIdleMs");
    assertEquals(0, shooting, 0.01);
    assertEquals(0, intaking, 0.01);
    assertEquals(0, aiming, 0.01);
    assertEquals(0, idle, 0.01);
  }

  @Test
  void testDefaultUptimeZero() throws Exception {
    telemetry.update();
    double uptime = getField(telemetry, "shooterUptimeMs");
    double lockTime = getField(telemetry, "visionLockTimeMs");
    assertEquals(0, uptime, 0.01);
    assertEquals(0, lockTime, 0.01);
  }

  @Test
  void testResetClearsAllStats() throws Exception {
    // Do some updates to accumulate state
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    telemetry.update();
    telemetry.update();

    telemetry.reset();

    int auto = getField(telemetry, "autoShots");
    int teleop = getField(telemetry, "teleopShots");
    int endgame = getField(telemetry, "endgameShots");
    double shootingMs = getField(telemetry, "timeShootingMs");
    double uptimeMs = getField(telemetry, "shooterUptimeMs");
    boolean inEndgame = getField(telemetry, "inEndgame");

    assertEquals(0, auto);
    assertEquals(0, teleop);
    assertEquals(0, endgame);
    assertEquals(0, shootingMs, 0.01);
    assertEquals(0, uptimeMs, 0.01);
    assertFalse(inEndgame);
  }

  @Test
  void testSkipsUpdateWhenDisabled() throws Exception {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    telemetry.update();

    // Should early-return, not accumulate time
    double idle = getField(telemetry, "timeIdleMs");
    assertEquals(0, idle, 0.01);
  }

  @Test
  void testMatchStartDetectedOnEnable() throws Exception {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    telemetry.update();

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double startTime = getField(telemetry, "matchStartTime");
    assertTrue(startTime > 0, "Match start time should be set when enabled");
  }

  @Test
  void testIdleTimeAccumulates() throws Exception {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    // CycleTracker defaults to IDLE phase
    telemetry.update();
    try {
      Thread.sleep(25);
    } catch (InterruptedException e) {
    }
    telemetry.update();

    double idle = getField(telemetry, "timeIdleMs");
    assertTrue(idle > 0, "Idle time should accumulate when enabled in IDLE phase");
  }

  @Test
  void testMultipleUpdatesDoNotThrow() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    assertDoesNotThrow(
        () -> {
          for (int i = 0; i < 10; i++) {
            telemetry.update();
            telemetry.log();
          }
        });
  }

  @Test
  void testNotInEndgameByDefault() throws Exception {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    telemetry.update();

    boolean inEndgame = getField(telemetry, "inEndgame");
    assertFalse(inEndgame, "Should not be in endgame at start");
  }

  @Test
  void testDefaultShiftShotsZero() throws Exception {
    telemetry.update();
    assertEquals(0, (int) getField(telemetry, "shotsShift1"));
    assertEquals(0, (int) getField(telemetry, "shotsShift2"));
    assertEquals(0, (int) getField(telemetry, "shotsShift3"));
    assertEquals(0, (int) getField(telemetry, "shotsShift4"));
    assertEquals(0, (int) getField(telemetry, "activeHubShots"));
    assertEquals(0, (int) getField(telemetry, "inactiveHubShots"));
    assertEquals(0, (double) getField(telemetry, "activeHubTimeMs"), 0.01);
    assertEquals(0, (double) getField(telemetry, "firingDuringActiveMs"), 0.01);
  }

  @Test
  void testResetClearsShiftStats() throws Exception {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    telemetry.update();
    telemetry.update();

    telemetry.reset();

    assertEquals(0, (int) getField(telemetry, "shotsShift1"));
    assertEquals(0, (int) getField(telemetry, "shotsShift2"));
    assertEquals(0, (int) getField(telemetry, "shotsShift3"));
    assertEquals(0, (int) getField(telemetry, "shotsShift4"));
    assertEquals(0, (int) getField(telemetry, "shotsShiftEndgame"));
    assertEquals(0, (int) getField(telemetry, "activeHubShots"));
    assertEquals(0, (int) getField(telemetry, "inactiveHubShots"));
    assertEquals(0, (double) getField(telemetry, "activeHubTimeMs"), 0.01);
    assertEquals(0, (double) getField(telemetry, "firingDuringActiveMs"), 0.01);
  }

  @Test
  void testDefaultEndgameShiftShotsZero() throws Exception {
    telemetry.update();
    assertEquals(0, (int) getField(telemetry, "shotsShiftEndgame"));
  }

  @Test
  void testHubMetricsSkipWhenScoringNull() {
    // Constructor already passes null for scoring,just verify no exceptions
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    assertDoesNotThrow(
        () -> {
          for (int i = 0; i < 5; i++) {
            telemetry.update();
            telemetry.log();
          }
        });
  }

  @Test
  void testHubUtilizationZeroByDefault() throws Exception {
    telemetry.update();
    telemetry.log();
    // activeHubTimeMs is 0 so hubUtil = 0
    double activeTime = getField(telemetry, "activeHubTimeMs");
    assertEquals(0, activeTime, 0.01);
  }
}
