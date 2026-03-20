package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.FireAuthorization.AuthorizationLevel;
import frc.robot.util.HubShiftEngine.ScheduleConfidence;
import frc.robot.util.HubShiftEngine.ShiftInfo;
import frc.robot.util.HubShiftEngine.ShiftPhase;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class FireAuthorizationTest {

  // mirrored from HubTimingConstants for readability in assertions
  private static final double AVG_FUEL_DELAY = 1.5;
  private static final double SAFE_MARGIN = 0.5;
  private static final double EXTENSION = 3.0;
  private static final double PRE_SPIN_WINDOW = 3.0;
  private static final double FALLBACK_EXTRA = 1.0;

  @BeforeEach
  void setUp() {
    HAL.initialize(500, 0);
    NetworkTableInstance.getDefault().startLocal();
    FireAuthorization.resetInstance();
    HubShiftEngine.resetInstance();
  }

  private ShiftInfo activeInfo(double remainingInState, double timeUntilDeactivation) {
    return new ShiftInfo(ShiftPhase.SHIFT2, true, 5.0, remainingInState, 0, timeUntilDeactivation);
  }

  private ShiftInfo activeInfoSimple(double timeUntilDeactivation) {
    return new ShiftInfo(
        ShiftPhase.SHIFT2, true, 5.0, timeUntilDeactivation, 0, timeUntilDeactivation);
  }

  private ShiftInfo inactiveInfo(double timeToNextActive) {
    return new ShiftInfo(ShiftPhase.SHIFT1, false, 5.0, 20.0, timeToNextActive, Double.MAX_VALUE);
  }

  @Test
  void testDefaultFailOpenBeforeFirstUpdate() {
    FireAuthorization fa = FireAuthorization.getInstance();
    assertEquals(
        AuthorizationLevel.FIRE_AUTHORIZED,
        fa.getLevel(),
        "Fail-open: default must be FIRE_AUTHORIZED before first update()");
    assertTrue(fa.isAuthorized(), "Fail-open: default must be authorized");
    assertFalse(fa.isPreSpin(), "Fail-open: must not be PRE_SPIN");
  }

  @Test
  void testKillSwitchAlwaysAuthorizes() {
    FireAuthorization fa = FireAuthorization.getInstance();
    fa.update(inactiveInfo(15.0), 1.0, ScheduleConfidence.HIGH, false);

    assertEquals(AuthorizationLevel.FIRE_AUTHORIZED, fa.getLevel());
    assertTrue(fa.isAuthorized());
  }

  @Test
  void testKillSwitchFieldValues() {
    FireAuthorization fa = FireAuthorization.getInstance();
    fa.update(inactiveInfo(15.0), 2.0, ScheduleConfidence.HIGH, false);

    assertEquals(Double.MAX_VALUE, fa.getMargin(), "Kill switch must set margin to MAX_VALUE");
    assertEquals(0.0, fa.getBallLandingTime(), 0.001, "Kill switch must set ballLandingTime to 0");
    assertEquals(
        Double.MAX_VALUE,
        fa.getWindowRemaining(),
        "Kill switch must set windowRemaining to MAX_VALUE");
  }

  @Test
  void testFireAuthorizedWithLargeMargin() {
    FireAuthorization fa = FireAuthorization.getInstance();
    fa.update(activeInfoSimple(20.0), 1.0, ScheduleConfidence.HIGH, true);

    assertEquals(AuthorizationLevel.FIRE_AUTHORIZED, fa.getLevel());
    assertTrue(fa.isAuthorized());
    assertTrue(fa.getMargin() > 10);
  }

  @Test
  void testFireAuthorizedExactBoundary() {
    FireAuthorization fa = FireAuthorization.getInstance();
    double tof = 1.0;
    double windowRemaining = SAFE_MARGIN + tof + AVG_FUEL_DELAY;
    fa.update(activeInfoSimple(windowRemaining), tof, ScheduleConfidence.HIGH, true);

    assertEquals(
        AuthorizationLevel.FIRE_AUTHORIZED,
        fa.getLevel(),
        "Exact safe margin boundary (margin=0.5) must be FIRE_AUTHORIZED, not" + " FIRE_MARGINAL");
    assertEquals(SAFE_MARGIN, fa.getMargin(), 0.001);
  }

  @Test
  void testFireMarginalInsideExtensionZone() {
    FireAuthorization fa = FireAuthorization.getInstance();
    fa.update(activeInfoSimple(2.0), 1.0, ScheduleConfidence.HIGH, true);

    assertEquals(AuthorizationLevel.FIRE_MARGINAL, fa.getLevel());
    assertTrue(fa.isAuthorized(), "FIRE_MARGINAL should still be authorized");
  }

  @Test
  void testExtensionBoundaryExact() {
    FireAuthorization fa = FireAuthorization.getInstance();
    // margin = 0.5 - (2.0 + 1.5) = -3.0
    fa.update(activeInfoSimple(0.5), 2.0, ScheduleConfidence.HIGH, true);

    assertEquals(-EXTENSION, fa.getMargin(), 0.001, "margin should be exactly -extension");
    assertEquals(
        AuthorizationLevel.HOLD_TIMING,
        fa.getLevel(),
        "At exact extension boundary, must be HOLD_TIMING not FIRE_MARGINAL");
    assertFalse(fa.isAuthorized());
  }

  @Test
  void testHoldTimingWhenPastExtension() {
    FireAuthorization fa = FireAuthorization.getInstance();
    fa.update(activeInfoSimple(0.0), 2.0, ScheduleConfidence.HIGH, true);

    assertEquals(AuthorizationLevel.HOLD_TIMING, fa.getLevel());
    assertFalse(fa.isAuthorized());
  }

  @Test
  void testPreSpinWhenWindowApproaching() {
    FireAuthorization fa = FireAuthorization.getInstance();
    fa.update(inactiveInfo(2.0), 1.0, ScheduleConfidence.HIGH, true);

    assertEquals(AuthorizationLevel.PRE_SPIN, fa.getLevel());
    assertTrue(fa.isPreSpin());
    assertFalse(fa.isAuthorized());
  }

  @Test
  void testPreSpinBoundaryExact() {
    FireAuthorization fa = FireAuthorization.getInstance();
    fa.update(inactiveInfo(PRE_SPIN_WINDOW), 1.0, ScheduleConfidence.HIGH, true);

    assertEquals(
        AuthorizationLevel.PRE_SPIN,
        fa.getLevel(),
        "At exact preSpinWindow (3.0s), must be PRE_SPIN not HOLD_INACTIVE");
    assertTrue(fa.isPreSpin());
  }

  @Test
  void testPreSpinJustOverBoundary() {
    FireAuthorization fa = FireAuthorization.getInstance();
    fa.update(inactiveInfo(PRE_SPIN_WINDOW + 0.001), 1.0, ScheduleConfidence.HIGH, true);

    assertEquals(
        AuthorizationLevel.HOLD_INACTIVE,
        fa.getLevel(),
        "timeToActive=3.001 > preSpinWindow=3.0 -> HOLD_INACTIVE");
    assertFalse(fa.isPreSpin());
  }

  @Test
  void testTimeToActiveZeroIsHoldInactive() {
    FireAuthorization fa = FireAuthorization.getInstance();
    fa.update(inactiveInfo(0.0), 1.0, ScheduleConfidence.HIGH, true);

    assertEquals(
        AuthorizationLevel.HOLD_INACTIVE,
        fa.getLevel(),
        "timeToActive=0 must be HOLD_INACTIVE (> 0 guard), not PRE_SPIN");
    assertFalse(fa.isPreSpin());
  }

  @Test
  void testHoldInactiveWhenFarFromWindow() {
    FireAuthorization fa = FireAuthorization.getInstance();
    fa.update(inactiveInfo(15.0), 1.0, ScheduleConfidence.HIGH, true);

    assertEquals(AuthorizationLevel.HOLD_INACTIVE, fa.getLevel());
    assertFalse(fa.isAuthorized());
    assertFalse(fa.isPreSpin());
  }

  @Test
  void testInactiveWindowRemainingIsZero() {
    FireAuthorization fa = FireAuthorization.getInstance();
    fa.update(activeInfoSimple(20.0), 1.0, ScheduleConfidence.HIGH, true);
    assertEquals(20.0, fa.getWindowRemaining(), 0.01, "Precondition: windowRemaining set to 20");

    fa.update(inactiveInfo(5.0), 1.0, ScheduleConfidence.HIGH, true);
    assertEquals(
        0.0,
        fa.getWindowRemaining(),
        0.001,
        "Inactive hub must set windowRemaining to 0, not carry over previous value");
  }

  @Test
  void testFallbackConfidenceWidensMargin() {
    FireAuthorization fa = FireAuthorization.getInstance();
    // margin = 3.0 - (1.0+1.5) = 0.5, but FALLBACK adds 1.0 to safe margin -> 0.5 < 1.5 ->
    // MARGINAL
    fa.update(activeInfoSimple(3.0), 1.0, ScheduleConfidence.FALLBACK, true);

    assertEquals(AuthorizationLevel.FIRE_MARGINAL, fa.getLevel());
    assertTrue(fa.isAuthorized(), "FIRE_MARGINAL should still authorize");
  }

  @Test
  void testHighConfidenceSameConditionIsAuthorized() {
    FireAuthorization fa = FireAuthorization.getInstance();
    fa.update(activeInfoSimple(3.0), 1.0, ScheduleConfidence.HIGH, true);

    assertEquals(AuthorizationLevel.FIRE_AUTHORIZED, fa.getLevel());
  }

  @Test
  void testBallLandingTimeComputation() {
    FireAuthorization fa = FireAuthorization.getInstance();
    fa.update(activeInfoSimple(20.0), 1.2, ScheduleConfidence.HIGH, true);

    assertEquals(2.7, fa.getBallLandingTime(), 0.01);
  }

  @Test
  void testWindowRemainingUsesTimeUntilDeactivation() {
    FireAuthorization fa = FireAuthorization.getInstance();
    double remainingInState = 8.0;
    double timeUntilDeactivation = 12.5;
    fa.update(
        activeInfo(remainingInState, timeUntilDeactivation), 1.0, ScheduleConfidence.HIGH, true);

    assertEquals(
        timeUntilDeactivation,
        fa.getWindowRemaining(),
        0.01,
        "windowRemaining must use timeUntilDeactivation (12.5), not remainingInState" + " (8.0)");
  }

  @Test
  void testTransitionAlwaysActive() {
    FireAuthorization fa = FireAuthorization.getInstance();
    ShiftInfo transitionInfo = new ShiftInfo(ShiftPhase.TRANSITION, true, 3.0, 7.0, 0, 7.0);
    fa.update(transitionInfo, 1.0, ScheduleConfidence.HIGH, true);

    assertEquals(AuthorizationLevel.FIRE_AUTHORIZED, fa.getLevel());
    assertTrue(fa.isAuthorized());
  }

  @Test
  void testEndgameAlwaysActive() {
    FireAuthorization fa = FireAuthorization.getInstance();
    ShiftInfo endgameInfo = new ShiftInfo(ShiftPhase.ENDGAME, true, 10.0, 20.0, 0, 20.0);
    fa.update(endgameInfo, 1.0, ScheduleConfidence.HIGH, true);

    assertEquals(AuthorizationLevel.FIRE_AUTHORIZED, fa.getLevel());
    assertTrue(fa.isAuthorized());
  }

  @Test
  void testStateTransitionActiveToInactive() {
    FireAuthorization fa = FireAuthorization.getInstance();

    fa.update(activeInfoSimple(20.0), 1.0, ScheduleConfidence.HIGH, true);
    assertTrue(fa.isAuthorized());

    fa.update(inactiveInfo(5.0), 1.0, ScheduleConfidence.HIGH, true);
    assertFalse(fa.isAuthorized(), "Should not be authorized after switching to inactive");
    assertEquals(0.0, fa.getWindowRemaining(), 0.001);
  }

  @Test
  void testKillSwitchToEnabledTransition() {
    FireAuthorization fa = FireAuthorization.getInstance();

    fa.update(inactiveInfo(15.0), 1.0, ScheduleConfidence.HIGH, false);
    assertEquals(Double.MAX_VALUE, fa.getMargin());

    fa.update(inactiveInfo(2.0), 1.0, ScheduleConfidence.HIGH, true);
    assertNotEquals(Double.MAX_VALUE, fa.getMargin());
    assertEquals(AuthorizationLevel.PRE_SPIN, fa.getLevel());
  }
}
