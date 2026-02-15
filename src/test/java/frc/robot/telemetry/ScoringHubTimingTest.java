package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.reflect.Method;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ScoringHubTimingTest extends TelemetryTestBase {

  private ScoringTelemetry scoring;

  // Reflected private methods for direct hub timing tests
  private Method computeShiftNumber;
  private Method computeHubActive;
  private Method computeTimeToNextShift;

  @BeforeEach
  void setUp() throws Exception {
    NetworkTableInstance.getDefault().startLocal();
    scoring = new ScoringTelemetry(null, null, null);

    // Access private hub timing methods via reflection
    computeShiftNumber =
        ScoringTelemetry.class.getDeclaredMethod("computeShiftNumber", double.class);
    computeShiftNumber.setAccessible(true);

    computeHubActive =
        ScoringTelemetry.class.getDeclaredMethod("computeHubActive", double.class, boolean.class);
    computeHubActive.setAccessible(true);

    computeTimeToNextShift =
        ScoringTelemetry.class.getDeclaredMethod("computeTimeToNextShift", double.class);
    computeTimeToNextShift.setAccessible(true);
  }

  @Test
  void testShiftNumberTransition() throws Exception {
    assertEquals(0, computeShiftNumber.invoke(scoring, 133.0), "matchTime>130 = transition");
    assertEquals(0, computeShiftNumber.invoke(scoring, 131.0), "matchTime=131 = transition");
  }

  @Test
  void testShiftNumber1() throws Exception {
    assertEquals(1, computeShiftNumber.invoke(scoring, 130.0), "matchTime=130 = shift 1");
    assertEquals(1, computeShiftNumber.invoke(scoring, 120.0), "matchTime=120 = shift 1");
    assertEquals(1, computeShiftNumber.invoke(scoring, 106.0), "matchTime=106 = shift 1");
  }

  @Test
  void testShiftNumber2() throws Exception {
    assertEquals(2, computeShiftNumber.invoke(scoring, 105.0), "matchTime=105 = shift 2");
    assertEquals(2, computeShiftNumber.invoke(scoring, 100.0), "matchTime=100 = shift 2");
    assertEquals(2, computeShiftNumber.invoke(scoring, 81.0), "matchTime=81 = shift 2");
  }

  @Test
  void testShiftNumber3() throws Exception {
    assertEquals(3, computeShiftNumber.invoke(scoring, 80.0), "matchTime=80 = shift 3");
    assertEquals(3, computeShiftNumber.invoke(scoring, 70.0), "matchTime=70 = shift 3");
    assertEquals(3, computeShiftNumber.invoke(scoring, 56.0), "matchTime=56 = shift 3");
  }

  @Test
  void testShiftNumber4() throws Exception {
    assertEquals(4, computeShiftNumber.invoke(scoring, 55.0), "matchTime=55 = shift 4");
    assertEquals(4, computeShiftNumber.invoke(scoring, 40.0), "matchTime=40 = shift 4");
    assertEquals(4, computeShiftNumber.invoke(scoring, 31.0), "matchTime=31 = shift 4");
  }

  @Test
  void testShiftNumberEndgame() throws Exception {
    assertEquals(0, computeShiftNumber.invoke(scoring, 30.0), "matchTime=30 = endgame");
    assertEquals(0, computeShiftNumber.invoke(scoring, 20.0), "matchTime=20 = endgame");
    assertEquals(0, computeShiftNumber.invoke(scoring, 0.0), "matchTime=0 = endgame");
  }

  @Test
  void testTransitionAlwaysActive() throws Exception {
    assertTrue((boolean) computeHubActive.invoke(scoring, 133.0, true));
    assertTrue((boolean) computeHubActive.invoke(scoring, 133.0, false));
  }

  @Test
  void testEndgameAlwaysActive() throws Exception {
    assertTrue((boolean) computeHubActive.invoke(scoring, 30.0, true));
    assertTrue((boolean) computeHubActive.invoke(scoring, 30.0, false));
    assertTrue((boolean) computeHubActive.invoke(scoring, 20.0, true));
    assertTrue((boolean) computeHubActive.invoke(scoring, 0.0, false));
  }

  @Test
  void testShift1WonAuto() throws Exception {
    // Odd shift: winner INACTIVE
    assertFalse(
        (boolean) computeHubActive.invoke(scoring, 120.0, true),
        "Winner should be INACTIVE during odd shift 1");
  }

  @Test
  void testShift1LostAuto() throws Exception {
    // Odd shift: loser ACTIVE
    assertTrue(
        (boolean) computeHubActive.invoke(scoring, 120.0, false),
        "Loser should be ACTIVE during odd shift 1");
  }

  @Test
  void testShift2WonAuto() throws Exception {
    // Even shift: winner ACTIVE
    assertTrue(
        (boolean) computeHubActive.invoke(scoring, 100.0, true),
        "Winner should be ACTIVE during even shift 2");
  }

  @Test
  void testShift2LostAuto() throws Exception {
    // Even shift: loser INACTIVE
    assertFalse(
        (boolean) computeHubActive.invoke(scoring, 100.0, false),
        "Loser should be INACTIVE during even shift 2");
  }

  @Test
  void testShift3WonAuto() throws Exception {
    // Odd shift: winner INACTIVE
    assertFalse(
        (boolean) computeHubActive.invoke(scoring, 70.0, true),
        "Winner should be INACTIVE during odd shift 3");
  }

  @Test
  void testShift3LostAuto() throws Exception {
    assertTrue(
        (boolean) computeHubActive.invoke(scoring, 70.0, false),
        "Loser should be ACTIVE during odd shift 3");
  }

  @Test
  void testShift4WonAuto() throws Exception {
    // Even shift: winner ACTIVE
    assertTrue(
        (boolean) computeHubActive.invoke(scoring, 40.0, true),
        "Winner should be ACTIVE during even shift 4");
  }

  @Test
  void testShift4LostAuto() throws Exception {
    assertFalse(
        (boolean) computeHubActive.invoke(scoring, 40.0, false),
        "Loser should be INACTIVE during even shift 4");
  }

  @Test
  void testTimeToNextShift() throws Exception {
    // In transition at matchTime=133, next shift at 130 → 3 seconds
    double timeToShift = (double) computeTimeToNextShift.invoke(scoring, 133.0);
    assertEquals(3.0, timeToShift, 0.01);

    // In shift 1 at matchTime=115, next shift at 105 → 10 seconds
    timeToShift = (double) computeTimeToNextShift.invoke(scoring, 115.0);
    assertEquals(10.0, timeToShift, 0.01);

    // In endgame at matchTime=20, time remaining = 20
    timeToShift = (double) computeTimeToNextShift.invoke(scoring, 20.0);
    assertEquals(20.0, timeToShift, 0.01);
  }

  @Test
  void testNullDependenciesSetsDefaults() throws Exception {
    setTeleopMode(120.0, false);
    scoring.update();

    boolean scoringAvailable = getField(scoring, "scoringAvailable");
    boolean readyToShoot = getField(scoring, "readyToShoot");
    assertFalse(scoringAvailable, "Scoring should be unavailable with null deps");
    assertFalse(readyToShoot, "ReadyToShoot should be false with null deps");
  }

  @Test
  void testAutoModeWithNullDeps() throws Exception {
    // Auto mode: hubActive defaults to true (field initializer)
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setTest(false);
    DriverStationSim.setMatchTime(120.0);
    DriverStationSim.notifyNewData();

    scoring.update();

    // With null deps, update() returns early. hubActive stays at initial value (true)
    boolean hubActive = getField(scoring, "hubActive");
    assertTrue(hubActive, "hubActive default should be true");
  }

  @Test
  void testUpdateAndLogDoNotThrow() {
    setTeleopMode(100.0, true);
    assertDoesNotThrow(
        () -> {
          scoring.update();
          scoring.log();
        });
  }

  @Test
  void testGetNameReturnsScoring() {
    assertEquals("Scoring", scoring.getName());
  }

  @Test
  void testIsReadyToShootAccessor() {
    scoring.update();
    assertFalse(scoring.isReadyToShoot(), "ReadyToShoot should be false with null deps");
  }

  private void setTeleopMode(double matchTime, boolean wonAuto) {
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setMatchTime(matchTime);
    DriverStationSim.notifyNewData();
    SmartDashboard.putBoolean("WonAuto", wonAuto);
  }
}
