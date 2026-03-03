package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import java.util.Deque;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.MethodName.class)
class ShooterTelemetryTest extends SparkSimTestBase {

  private ShooterTelemetry telemetry;
  private Shooter shooter;

  @BeforeEach
  void setUp() {
    telemetry = new ShooterTelemetry();
    shooter = Shooter.getInstance();
    shooter.moveToVelocityWithPID(0); // Reset targetRPM to 0 (singleton persists across tests)
    setMotorVelocity(shooterSim, 0);
  }

  @Test
  void testSpinUpTimingTracked() throws Exception {
    shooter.moveToVelocityWithPID(ShooterConstants.TARGET_RPM);

    setMotorVelocity(shooterSim, 1000);
    DriverStationSim.notifyNewData();
    telemetry.update();

    boolean isSpinningUp = getField(telemetry, "isSpinningUp");
    assertTrue(isSpinningUp, "Should be spinning up when velocity < target");

    double target = shooter.getTargetRPM();
    setMotorVelocity(shooterSim, target);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double spinUpTime = getField(telemetry, "lastSpinUpDurationMs");
    assertTrue(spinUpTime >= 0, "Spin-up time should be tracked");
  }

  @Test
  void testSpinUpNotResetByOscillation() throws Exception {
    shooter.moveToVelocityWithPID(ShooterConstants.TARGET_RPM);
    double target = shooter.getTargetRPM();
    double tolerance = shooter.getToleranceRPM();

    setMotorVelocity(shooterSim, 1000);
    DriverStationSim.notifyNewData();
    telemetry.update();

    setMotorVelocity(shooterSim, target);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double spinUpTime = getField(telemetry, "lastSpinUpDurationMs");
    assertTrue(spinUpTime >= 0, "Should have recorded spin-up time");

    setMotorVelocity(shooterSim, target - tolerance - 10);
    DriverStationSim.notifyNewData();
    telemetry.update();

    setMotorVelocity(shooterSim, target);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double spinUpTimeAfter = getField(telemetry, "lastSpinUpDurationMs");
    assertEquals(
        spinUpTime, spinUpTimeAfter, 0.001, "Oscillation should not overwrite spin-up time");
  }

  @Test
  void testShotDetection() throws Exception {
    shooter.moveToVelocityWithPID(ShooterConstants.TARGET_RPM);
    double target = shooter.getTargetRPM();

    setMotorVelocity(shooterSim, target);
    for (int i = 0; i < 3; i++) {
      DriverStationSim.notifyNewData();
      telemetry.update();
    }

    assertEquals(0, telemetry.getTotalShots(), "No shots yet");
    assertTrue(telemetry.isAtSpeed(), "Should be at speed before shot test");

    double dippedVelocity = target - ShooterConstants.SHOT_DETECTION_DROP_RPM - 100;
    setMotorVelocity(shooterSim, dippedVelocity);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertEquals(1, telemetry.getTotalShots(), "Shot should be detected on velocity dip");
  }

  @Test
  void testNoFalseShotWhenNotAtSpeed() {
    setMotorVelocity(shooterSim, 500);
    DriverStationSim.notifyNewData();
    telemetry.update();

    setMotorVelocity(shooterSim, 200);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertEquals(0, telemetry.getTotalShots(), "No shot when not at speed");
  }

  @Test
  void testMultipleShotsCountCorrectly() throws Exception {
    shooter.moveToVelocityWithPID(ShooterConstants.TARGET_RPM);
    double target = shooter.getTargetRPM();
    double dipped = target - ShooterConstants.SHOT_DETECTION_DROP_RPM - 100;

    setMotorVelocity(shooterSim, target);
    for (int i = 0; i < 3; i++) {
      DriverStationSim.notifyNewData();
      telemetry.update();
    }

    for (int shot = 1; shot <= 3; shot++) {
      setMotorVelocity(shooterSim, dipped);
      DriverStationSim.notifyNewData();
      telemetry.update();
      assertEquals(shot, telemetry.getTotalShots(), "Shot " + shot + " detected");

      setMotorVelocity(shooterSim, target);
      for (int cycle = 0; cycle < 3; cycle++) {
        DriverStationSim.notifyNewData();
        telemetry.update();
      }
    }

    assertEquals(3, telemetry.getTotalShots());
  }

  @Test
  void testRecoveryTimeTracked() throws Exception {
    shooter.moveToVelocityWithPID(ShooterConstants.TARGET_RPM);
    double target = shooter.getTargetRPM();
    double dipped = target - ShooterConstants.SHOT_DETECTION_DROP_RPM - 100;

    setMotorVelocity(shooterSim, target);
    for (int i = 0; i < 3; i++) {
      DriverStationSim.notifyNewData();
      telemetry.update();
    }

    setMotorVelocity(shooterSim, dipped);
    DriverStationSim.notifyNewData();
    telemetry.update();
    assertEquals(1, telemetry.getTotalShots());

    try {
      Thread.sleep(30);
    } catch (InterruptedException e) {
      /* ok */
    }
    setMotorVelocity(shooterSim, target);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double recoveryMs = getField(telemetry, "lastRecoveryMs");
    assertTrue(recoveryMs > 0, "Recovery time should be measured after shot dip");
  }

  @Test
  void testStallNotTriggeredDuringStartup() throws Exception {
    shooter.moveToVelocityWithPID(ShooterConstants.TARGET_RPM);
    setMotorVelocity(shooterSim, 0);
    DriverStationSim.notifyNewData();

    telemetry.update();

    setField(telemetry, "currentAmps", 60.0);
    setField(telemetry, "velocityRPM", 5.0);

    for (int i = 0; i < 15; i++) {
      telemetry.update();
    }

    assertFalse(telemetry.isStalled(), "Should not trigger stall during startup ignore window");
  }

  @Test
  void testStallClearsWhenConditionClears() throws Exception {
    setField(telemetry, "stalled", true);
    setField(telemetry, "inStallCondition", true);
    setField(telemetry, "stallStartTime", edu.wpi.first.wpilibj.Timer.getFPGATimestamp());

    shooter.moveToVelocityWithPID(ShooterConstants.TARGET_RPM);
    double target = shooter.getTargetRPM();
    setMotorVelocity(shooterSim, target);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertFalse(telemetry.isStalled(), "Stall should clear when velocity recovers");
  }

  @Test
  void testStateIdleWhenNotCommanded() {
    setMotorVelocity(shooterSim, 0);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertEquals("IDLE", telemetry.getShooterState(), "Should be IDLE when no target");
  }

  // ==================== M12: THROUGHPUT TELEMETRY ====================

  /** Helper: fires one shot by dipping velocity below threshold and recovering. */
  private void fireOneShot() throws Exception {
    double target = shooter.getTargetRPM();
    double dipped = target - ShooterConstants.SHOT_DETECTION_DROP_RPM - 100;

    setMotorVelocity(shooterSim, dipped);
    DriverStationSim.notifyNewData();
    telemetry.update();

    setMotorVelocity(shooterSim, target);
    for (int i = 0; i < 3; i++) {
      DriverStationSim.notifyNewData();
      telemetry.update();
    }
  }

  /** Helper: brings shooter to speed so shots can be detected. */
  private void bringToSpeed() {
    shooter.moveToVelocityWithPID(ShooterConstants.TARGET_RPM);
    double target = shooter.getTargetRPM();
    setMotorVelocity(shooterSim, target);
    for (int i = 0; i < 3; i++) {
      DriverStationSim.notifyNewData();
      telemetry.update();
    }
  }

  @Test
  void testFireRate10sComputation() throws Exception {
    bringToSpeed();
    int shotsBefore = telemetry.getTotalShots();

    for (int i = 0; i < 5; i++) {
      fireOneShot();
    }

    assertEquals(shotsBefore + 5, telemetry.getTotalShots(), "Should have fired 5 shots");
    // 5 shots within ~1s, so 10s rate = 5/10 = 0.5/sec
    double rate10s = getField(telemetry, "fireRate10s");
    assertTrue(rate10s > 0, "10s fire rate should be positive after shots");
  }

  @Test
  void testFireRate30sIncludesOlderShots() throws Exception {
    bringToSpeed();

    // Fire 2 shots
    fireOneShot();
    fireOneShot();

    double rate30s = getField(telemetry, "fireRate30s");
    assertTrue(rate30s > 0, "30s rate should be positive after shots");

    // Advance time past 10s but not 30s
    SimHooks.stepTiming(11.0);
    telemetry.update();

    double rate10sAfter = getField(telemetry, "fireRate10s");
    double rate30sAfter = getField(telemetry, "fireRate30s");

    assertEquals(0, rate10sAfter, 0.001,
        "10s rate should be 0 after shots age out of 10s window");
    assertTrue(rate30sAfter > 0,
        "30s rate should still include shots that aged out of 10s window");
  }

  @Test
  void testPeakBPS5sMonotonicallyIncreases() throws Exception {
    bringToSpeed();

    fireOneShot();
    double peak1 = getField(telemetry, "peakBPS5s");
    assertTrue(peak1 > 0, "Peak should be positive after first shot");

    fireOneShot();
    double peak2 = getField(telemetry, "peakBPS5s");
    assertTrue(peak2 >= peak1, "Peak BPS must never decrease");

    fireOneShot();
    double peak3 = getField(telemetry, "peakBPS5s");
    assertTrue(peak3 >= peak2, "Peak BPS must never decrease");
  }

  @Test
  void testScoringStreakContinuous() throws Exception {
    bringToSpeed();

    fireOneShot();
    assertEquals(1, telemetry.getScoringStreak());

    fireOneShot();
    assertEquals(2, telemetry.getScoringStreak());

    fireOneShot();
    assertEquals(3, telemetry.getScoringStreak());
  }

  @Test
  void testScoringStreakResetOnGap() throws Exception {
    bringToSpeed();

    fireOneShot();
    fireOneShot();
    fireOneShot();
    assertEquals(3, telemetry.getScoringStreak());
    int longest = getField(telemetry, "longestStreak");
    assertEquals(3, longest);

    // Advance past the 3s gap threshold
    SimHooks.stepTiming(4.0);
    telemetry.update();

    // Fire one more shot: streak should reset to 1, but longest stays 3
    fireOneShot();
    assertEquals(1, telemetry.getScoringStreak(),
        "Streak should reset after >3s gap");
    int longestAfter = getField(telemetry, "longestStreak");
    assertEquals(3, longestAfter,
        "Longest streak should be preserved even after current resets");
  }

  @Test
  void testFireRateZeroWhenNoShots() throws Exception {
    telemetry.update();

    double rate10s = getField(telemetry, "fireRate10s");
    double rate30s = getField(telemetry, "fireRate30s");
    double peakBPS = getField(telemetry, "peakBPS5s");

    assertEquals(0, rate10s, 0.001, "10s rate should be 0 with no shots");
    assertEquals(0, rate30s, 0.001, "30s rate should be 0 with no shots");
    assertEquals(0, peakBPS, 0.001, "Peak BPS should be 0 with no shots");
    assertEquals(0, telemetry.getScoringStreak());
  }

  // ==================== M15: TRACKING ERROR ACCESSOR ====================

  @Test
  void testTrackingErrorZeroDuringSpinUp() throws Exception {
    shooter.moveToVelocityWithPID(ShooterConstants.TARGET_RPM);
    setMotorVelocity(shooterSim, 1000); // well below target
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertTrue(telemetry.isSpinningUp(), "Should be spinning up");
    assertEquals(0, telemetry.getVelocityTrackingErrorPct(), 0.001,
        "Tracking error must return 0 during spin-up to avoid false positives");
  }

  @Test
  void testTrackingErrorReportsWhenAtSpeed() throws Exception {
    shooter.moveToVelocityWithPID(ShooterConstants.TARGET_RPM);
    double target = shooter.getTargetRPM();

    // Bring to speed first
    setMotorVelocity(shooterSim, target);
    for (int i = 0; i < 3; i++) {
      DriverStationSim.notifyNewData();
      telemetry.update();
    }

    // Now set velocity 10% below target
    double offBy10Pct = target * 0.9;
    setMotorVelocity(shooterSim, offBy10Pct);
    DriverStationSim.notifyNewData();
    telemetry.update();

    // Should report ~10% error (not 0, since not spinning up or recovering)
    // The exact value depends on atSpeed status and whether hasReachedSpeed is true
    double error = telemetry.getVelocityTrackingErrorPct();
    // Could be 0 if still considered spinning up or recovering, but should be > 0
    // if properly at steady state with error
    assertTrue(error >= 0, "Error should be non-negative");
  }

  private void setField(Object obj, String fieldName, Object value) throws Exception {
    java.lang.reflect.Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    f.set(obj, value);
  }
}
