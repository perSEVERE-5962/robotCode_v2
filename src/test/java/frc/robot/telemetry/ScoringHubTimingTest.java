package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.reflect.Method;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ScoringHubTimingTest extends TelemetryTestBase {

  private ScoringTelemetry scoring;

  private Method computeShiftNumber;
  private Method computeHubActive;
  private Method computeTimeToNextShift;

  @BeforeEach
  void setUp() throws Exception {
    NetworkTableInstance.getDefault().startLocal();
    scoring = new ScoringTelemetry(null, null, null);

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
  void testShiftNumber1() throws Exception {
    assertEquals(1, computeShiftNumber.invoke(scoring, 130.0), "matchTime=130 = shift 1");
    assertEquals(1, computeShiftNumber.invoke(scoring, 106.0), "matchTime=106 = shift 1");
  }

  @Test
  void testShiftNumber4() throws Exception {
    assertEquals(4, computeShiftNumber.invoke(scoring, 55.0), "matchTime=55 = shift 4");
    assertEquals(4, computeShiftNumber.invoke(scoring, 31.0), "matchTime=31 = shift 4");
  }

  @Test
  void testShiftNumberEndgame() throws Exception {
    assertEquals(0, computeShiftNumber.invoke(scoring, 30.0), "matchTime=30 = endgame");
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
    assertTrue((boolean) computeHubActive.invoke(scoring, 0.0, false));
  }

  @Test
  void testShift1WonAuto() throws Exception {
    assertFalse(
        (boolean) computeHubActive.invoke(scoring, 120.0, true),
        "Winner should be INACTIVE during odd shift 1");
  }

  @Test
  void testShift1LostAuto() throws Exception {
    assertTrue(
        (boolean) computeHubActive.invoke(scoring, 120.0, false),
        "Loser should be ACTIVE during odd shift 1");
  }

  @Test
  void testShift2WonAuto() throws Exception {
    assertTrue(
        (boolean) computeHubActive.invoke(scoring, 100.0, true),
        "Winner should be ACTIVE during even shift 2");
  }

  @Test
  void testTimeToNextShift() throws Exception {
    double timeToShift = (double) computeTimeToNextShift.invoke(scoring, 133.0);
    assertEquals(3.0, timeToShift, 0.01);

    timeToShift = (double) computeTimeToNextShift.invoke(scoring, 115.0);
    assertEquals(10.0, timeToShift, 0.01);

    timeToShift = (double) computeTimeToNextShift.invoke(scoring, 20.0);
    assertEquals(20.0, timeToShift, 0.01);
  }
}
