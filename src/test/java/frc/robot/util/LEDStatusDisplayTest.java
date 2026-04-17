package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.telemetry.SafeLog;
import frc.robot.util.LEDStatusDisplay.LEDState;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LEDStatusDisplayTest {

  private LEDStatusDisplay display;

  @BeforeEach
  void setUp() throws Exception {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    resetSingleton("frc.robot.util.LEDStatusDisplay");
    resetSingleton("frc.robot.util.DriverFeedback");
    resetSingleton("frc.robot.util.ChannelCoordinator");
    resetSingleton("frc.robot.telemetry.TelemetryManager");

    display = LEDStatusDisplay.getInstance();
  }

  @AfterEach
  void tearDown() throws Exception {
    resetSingleton("frc.robot.util.LEDStatusDisplay");
    resetSingleton("frc.robot.util.DriverFeedback");
    resetSingleton("frc.robot.util.ChannelCoordinator");
    resetSingleton("frc.robot.telemetry.TelemetryManager");

    SafeLog.logAndReset();
  }

  private static void resetSingleton(String className) {
    try {
      Class<?> clazz = Class.forName(className);
      Field f = clazz.getDeclaredField("instance");
      f.setAccessible(true);
      f.set(null, null);
    } catch (Exception e) {
    }
  }

  private LEDStatusDisplay.StateSnapshot snap(
      boolean enabled,
      boolean autonomous,
      boolean teleop,
      double matchTime,
      double batteryVoltage,
      boolean brownout,
      boolean readyToShoot,
      boolean progressiveAimActive,
      double progressiveAimError,
      boolean shooterSpinningUp,
      double shooterAtSpeedPercent,
      boolean jamDetected,
      boolean anyStalled,
      boolean batteryWarning,
      boolean canError,
      boolean visionLocked,
      boolean lowConfidence) {
    return new LEDStatusDisplay.StateSnapshot(
        enabled,
        autonomous,
        teleop,
        matchTime,
        batteryVoltage,
        brownout,
        readyToShoot,
        progressiveAimActive,
        progressiveAimError,
        shooterSpinningUp,
        shooterAtSpeedPercent,
        jamDetected,
        anyStalled,
        batteryWarning,
        canError,
        visionLocked,
        lowConfidence);
  }

  private LEDStatusDisplay.StateSnapshot defaultEnabled() {
    return snap(
        true, false, true, 120, 12.5, false, false, false, -1, false, 0, false, false, false, false,
        false, false);
  }

  private LEDStatusDisplay.StateSnapshot defaultDisabled() {
    return snap(
        false, false, false, -1, 12.5, false, false, false, -1, false, 0, false, false, false,
        false, false, false);
  }

  private LEDState resolve(LEDStatusDisplay.StateSnapshot s) throws Exception {
    Method m =
        LEDStatusDisplay.class.getDeclaredMethod(
            "resolveState", LEDStatusDisplay.StateSnapshot.class);
    m.setAccessible(true);
    return (LEDState) m.invoke(display, s);
  }

  @Test
  void testDisabledDefault() throws Exception {
    assertEquals(LEDState.DISABLED, resolve(defaultDisabled()));
  }

  @Test
  void testIdleWhenEnabled() throws Exception {
    assertEquals(LEDState.IDLE, resolve(defaultEnabled()));
  }

  @Test
  void testReadyToShootSolidBlue() throws Exception {
    var s =
        snap(
            true, false, true, 120, 12.5, false, true, false, -1, false, 100, false, false, false,
            false, true, false);
    assertEquals(LEDState.READY_TO_SHOOT, resolve(s));
  }

  @Test
  void testCriticalOverridesAll() throws Exception {
    var s =
        snap(
            true, false, true, 25, 9.5, false, true, false, -1, false, 100, true, true, true, true,
            true, false);
    assertEquals(LEDState.CRITICAL_ALERT, resolve(s));
  }

  @Test
  void testBrownoutTriggersCritical() throws Exception {
    var s =
        snap(
            true, false, true, 120, 11.0, true, false, false, -1, false, 0, false, false, false,
            false, false, false);
    assertEquals(LEDState.CRITICAL_ALERT, resolve(s));
  }

  @Test
  void testShooterSpinup() throws Exception {
    var s =
        snap(
            true, false, true, 120, 12.5, false, false, false, -1, true, 50, false, false, false,
            false, false, false);
    assertEquals(LEDState.SHOOTER_SPINUP, resolve(s));
  }

  @Test
  void testAimProgress() throws Exception {
    var s =
        snap(
            true, false, true, 120, 12.5, false, false, true, 5.0, false, 0, false, false, false,
            false, false, false);
    assertEquals(LEDState.AIM_PROGRESS, resolve(s));
  }

  @Test
  void testWarningOnJam() throws Exception {
    var s =
        snap(
            true, false, true, 120, 12.5, false, false, false, -1, false, 0, true, false, false,
            false, false, false);
    assertEquals(LEDState.WARNING, resolve(s));
  }

  @Test
  void testScoringVisibleDuringEndgame() throws Exception {
    var s =
        snap(
            true, false, true, 25, 12.5, false, true, false, -1, false, 100, false, false, false,
            false, true, false);
    assertEquals(LEDState.READY_TO_SHOOT, resolve(s));
  }

  @Test
  void testReadyToShootOverridesAim() throws Exception {
    var s =
        snap(
            true, false, true, 120, 12.5, false, true, true, 3.0, false, 100, false, false, false,
            false, true, false);
    assertEquals(LEDState.READY_TO_SHOOT, resolve(s));
  }

  @Test
  void testLowConfidenceTriggersWarning() throws Exception {
    var s =
        snap(
            true, false, true, 120, 12.5, false, false, false, -1, false, 0, false, false, false,
            false, false, true);
    assertEquals(LEDState.WARNING, resolve(s));
  }

  // ==================== DEACTIVATION TESTS ====================
  // Pattern 7 from FMEA: "If it turns on, prove it turns off"
  // Since resolveState() is a pure function recomputed every cycle, deactivation means
  // removing the triggering condition returns to a lower-priority state.

  @Test
  void testCriticalAlertDeactivatesToIdle() throws Exception {
    // Activate: brownout triggers CRITICAL_ALERT
    var critical =
        snap(
            true, false, true, 120, 11.0, true, false, false, -1, false, 0, false, false, false,
            false, false, false);
    assertEquals(LEDState.CRITICAL_ALERT, resolve(critical));

    // Deactivate: brownout clears, voltage normal -> IDLE
    assertEquals(LEDState.IDLE, resolve(defaultEnabled()));
  }

  @Test
  void testReadyToShootDeactivatesToIdle() throws Exception {
    // Activate
    var active =
        snap(
            true, false, true, 120, 12.5, false, true, false, -1, false, 100, false, false, false,
            false, true, false);
    assertEquals(LEDState.READY_TO_SHOOT, resolve(active));

    // Deactivate: readyToShoot=false -> IDLE
    assertEquals(LEDState.IDLE, resolve(defaultEnabled()));
  }

  @Test
  void testAimProgressDeactivatesToIdle() throws Exception {
    // Activate
    var active =
        snap(
            true, false, true, 120, 12.5, false, false, true, 5.0, false, 0, false, false, false,
            false, false, false);
    assertEquals(LEDState.AIM_PROGRESS, resolve(active));

    // Deactivate: progressiveAimActive=false -> IDLE
    assertEquals(LEDState.IDLE, resolve(defaultEnabled()));
  }

  @Test
  void testShooterSpinupDeactivatesToIdle() throws Exception {
    // Activate
    var active =
        snap(
            true, false, true, 120, 12.5, false, false, false, -1, true, 50, false, false, false,
            false, false, false);
    assertEquals(LEDState.SHOOTER_SPINUP, resolve(active));

    // Deactivate: shooterSpinningUp=false -> IDLE
    assertEquals(LEDState.IDLE, resolve(defaultEnabled()));
  }

  @Test
  void testWarningJamDeactivatesToIdle() throws Exception {
    // Activate via jam
    var active =
        snap(
            true, false, true, 120, 12.5, false, false, false, -1, false, 0, true, false, false,
            false, false, false);
    assertEquals(LEDState.WARNING, resolve(active));

    // Deactivate: jamDetected=false -> IDLE
    assertEquals(LEDState.IDLE, resolve(defaultEnabled()));
  }

  @Test
  void testWarningStallDeactivatesToIdle() throws Exception {
    // Activate via stall
    var active =
        snap(
            true, false, true, 120, 12.5, false, false, false, -1, false, 0, false, true, false,
            false, false, false);
    assertEquals(LEDState.WARNING, resolve(active));

    // Deactivate: anyStalled=false -> IDLE
    assertEquals(LEDState.IDLE, resolve(defaultEnabled()));
  }

  @Test
  void testWarningLowConfidenceDeactivatesToIdle() throws Exception {
    // Activate via lowConfidence
    var active =
        snap(
            true, false, true, 120, 12.5, false, false, false, -1, false, 0, false, false, false,
            false, false, true);
    assertEquals(LEDState.WARNING, resolve(active));

    // Deactivate: lowConfidence=false -> IDLE
    assertEquals(LEDState.IDLE, resolve(defaultEnabled()));
  }

  @Test
  void testAutoRunningDeactivatesToIdle() throws Exception {
    // Activate: autonomous=true
    var active =
        snap(
            true, true, false, 120, 12.5, false, false, false, -1, false, 0, false, false, false,
            false, false, false);
    assertEquals(LEDState.AUTO_RUNNING, resolve(active));

    // Deactivate: autonomous=false, teleop=true -> IDLE
    assertEquals(LEDState.IDLE, resolve(defaultEnabled()));
  }

  @Test
  void testVisionLockedDeactivatesToIdle() throws Exception {
    // Activate: visionLocked=true
    var active =
        snap(
            true, false, true, 120, 12.5, false, false, false, -1, false, 0, false, false, false,
            false, true, false);
    assertEquals(LEDState.VISION_LOCKED, resolve(active));

    // Deactivate: visionLocked=false -> IDLE
    assertEquals(LEDState.IDLE, resolve(defaultEnabled()));
  }

  @Test
  void testDisabledNeverShowsCritical() throws Exception {
    // Even with brownout, if disabled -> DISABLED (not CRITICAL_ALERT)
    var s =
        snap(
            false, false, false, -1, 9.5, true, false, false, -1, false, 0, false, false, false,
            false, false, false);
    assertEquals(LEDState.DISABLED, resolve(s));
  }
}
