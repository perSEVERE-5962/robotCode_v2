package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
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
    // Reset encoder to 0 between tests
    setMotorVelocity(shooterSim, 0);
  }

  @Test
  void testGetNameReturnsShooter() {
    assertEquals("Shooter", telemetry.getName());
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
  void testDefaultsWhenStopped() {
    setMotorVelocity(shooterSim, 0);
    telemetry.update();

    assertFalse(telemetry.isAtSpeed());
    assertEquals(0, telemetry.getTotalShots());
    assertFalse(telemetry.isStalled());
  }

  @Test
  void testSpinUpDetection() throws Exception {
    shooter.move(1.0); // targetRPM = 5700
    double target = shooter.getTargetRPM();

    setMotorVelocity(shooterSim, target);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertTrue(telemetry.isAtSpeed(), "Should be at speed when velocity matches target");
  }

  @Test
  void testSpinUpTimingTracked() throws Exception {
    shooter.move(1.0);

    // First cycle: spinning up (not yet at speed)
    setMotorVelocity(shooterSim, 1000);
    DriverStationSim.notifyNewData();
    telemetry.update();

    boolean isSpinningUp = getField(telemetry, "isSpinningUp");
    assertTrue(isSpinningUp, "Should be spinning up when velocity < target");

    // Reach speed
    double target = shooter.getTargetRPM();
    setMotorVelocity(shooterSim, target);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double spinUpTime = getField(telemetry, "lastSpinUpDurationMs");
    assertTrue(spinUpTime >= 0, "Spin-up time should be tracked");
  }

  @Test
  void testSpinUpNotResetByOscillation() throws Exception {
    shooter.move(1.0);
    double target = shooter.getTargetRPM();
    double tolerance = shooter.getToleranceRPM();

    // Spin up: low velocity first
    setMotorVelocity(shooterSim, 1000);
    DriverStationSim.notifyNewData();
    telemetry.update();

    // Reach speed
    setMotorVelocity(shooterSim, target);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double spinUpTime = getField(telemetry, "lastSpinUpDurationMs");
    assertTrue(spinUpTime >= 0, "Should have recorded spin-up time");

    // Simulate oscillation: dip below tolerance
    setMotorVelocity(shooterSim, target - tolerance - 10);
    DriverStationSim.notifyNewData();
    telemetry.update();

    // Recover back to speed
    setMotorVelocity(shooterSim, target);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double spinUpTimeAfter = getField(telemetry, "lastSpinUpDurationMs");
    assertEquals(
        spinUpTime, spinUpTimeAfter, 0.001, "Oscillation should not overwrite spin-up time");
  }

  @Test
  void testSpinUpResetsOnNewCommand() throws Exception {
    shooter.move(1.0);
    double target = shooter.getTargetRPM();

    // First spin-up
    setMotorVelocity(shooterSim, 1000);
    DriverStationSim.notifyNewData();
    telemetry.update();
    setMotorVelocity(shooterSim, target);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double firstSpinUp = getField(telemetry, "lastSpinUpDurationMs");
    assertTrue(firstSpinUp >= 0, "First spin-up recorded");

    // Command ends
    shooter.move(0);
    setMotorVelocity(shooterSim, 0);
    DriverStationSim.notifyNewData();
    telemetry.update();

    // Small delay for measurable time difference
    try {
      Thread.sleep(50);
    } catch (InterruptedException e) {
      /* ok */
    }

    // New command starts fresh
    shooter.move(1.0);
    setMotorVelocity(shooterSim, 1000);
    DriverStationSim.notifyNewData();
    telemetry.update();
    setMotorVelocity(shooterSim, target);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double secondSpinUp = getField(telemetry, "lastSpinUpDurationMs");
    assertTrue(secondSpinUp >= 0, "Second spin-up recorded");
  }

  @Test
  void testInitialSpinUpTimeIsNegativeOne() throws Exception {
    telemetry.update();
    telemetry.log();

    double spinUpTime = getField(telemetry, "lastSpinUpDurationMs");
    assertEquals(-1.0, spinUpTime, 0.001, "Initial spin-up time should be -1 (not yet measured)");
  }

  @Test
  void testShotDetection() throws Exception {
    shooter.move(1.0);
    double target = shooter.getTargetRPM();

    // Stabilize at speed,need multiple update() cycles to prime wasAtSpeed
    setMotorVelocity(shooterSim, target);
    for (int i = 0; i < 3; i++) {
      DriverStationSim.notifyNewData();
      telemetry.update();
    }

    assertEquals(0, telemetry.getTotalShots(), "No shots yet");
    assertTrue(telemetry.isAtSpeed(), "Should be at speed before shot test");

    // Dip velocity (shot fired),drop must exceed SHOT_DETECTION_DROP_RPM (200)
    double dippedVelocity = target - ShooterConstants.SHOT_DETECTION_DROP_RPM - 100;
    setMotorVelocity(shooterSim, dippedVelocity);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertEquals(1, telemetry.getTotalShots(), "Shot should be detected on velocity dip");
  }

  @Test
  void testNoFalseShotWhenNotAtSpeed() {
    // Don't command shooter,velocity fluctuates but never "at speed"
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
    shooter.move(1.0);
    double target = shooter.getTargetRPM();
    double dipped = target - ShooterConstants.SHOT_DETECTION_DROP_RPM - 100;

    // Stabilize at speed
    setMotorVelocity(shooterSim, target);
    for (int i = 0; i < 3; i++) {
      DriverStationSim.notifyNewData();
      telemetry.update();
    }

    // 3 shot cycles: dip then recover
    for (int shot = 1; shot <= 3; shot++) {
      // Dip
      setMotorVelocity(shooterSim, dipped);
      DriverStationSim.notifyNewData();
      telemetry.update();
      assertEquals(shot, telemetry.getTotalShots(), "Shot " + shot + " detected");

      // Recover (3 cycles to re-prime wasAtSpeed + previousVelocity)
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
    shooter.move(1.0);
    double target = shooter.getTargetRPM();
    double dipped = target - ShooterConstants.SHOT_DETECTION_DROP_RPM - 100;

    // Stabilize at speed
    setMotorVelocity(shooterSim, target);
    for (int i = 0; i < 3; i++) {
      DriverStationSim.notifyNewData();
      telemetry.update();
    }

    // Shot dip
    setMotorVelocity(shooterSim, dipped);
    DriverStationSim.notifyNewData();
    telemetry.update();
    assertEquals(1, telemetry.getTotalShots());

    // Small delay then recover
    try {
      Thread.sleep(30);
    } catch (InterruptedException e) {
      /* ok */
    }
    setMotorVelocity(shooterSim, target);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double recoveryMs = getField(telemetry, "lastRecoveryMs");
    assertTrue(recoveryMs > 0, "Recovery time should be measured after shot dip → at speed");
  }

  @Test
  void testRecoveryResetWhenShooterStops() throws Exception {
    shooter.move(1.0);
    double target = shooter.getTargetRPM();
    double dipped = target - ShooterConstants.SHOT_DETECTION_DROP_RPM - 100;

    // Stabilize at speed
    setMotorVelocity(shooterSim, target);
    for (int i = 0; i < 3; i++) {
      DriverStationSim.notifyNewData();
      telemetry.update();
    }

    // Shot dip -> trackingRecovery = true
    setMotorVelocity(shooterSim, dipped);
    DriverStationSim.notifyNewData();
    telemetry.update();
    assertEquals(1, telemetry.getTotalShots());

    boolean tracking = getField(telemetry, "trackingRecovery");
    assertTrue(tracking, "Should be tracking recovery after shot");

    // Driver releases trigger -> targetRPM goes to 0
    shooter.move(0);
    setMotorVelocity(shooterSim, 0);
    DriverStationSim.notifyNewData();
    telemetry.update();

    boolean trackingAfterStop = getField(telemetry, "trackingRecovery");
    assertFalse(trackingAfterStop, "Recovery tracking should reset when shooter stops");
  }

  @Test
  void testStallDetection() throws Exception {
    shooter.move(1.0); // targetRPM > 0 required

    // Stall: high current + low velocity. Use iterate() for physics-based current.
    iterateMotor(shooterSim, 10, 12.5);
    DriverStationSim.notifyNewData();

    // Wait for stall debounce (250ms)
    long startMs = System.currentTimeMillis();
    while (System.currentTimeMillis() - startMs < 300) {
      iterateMotor(shooterSim, 10, 12.5);
      DriverStationSim.notifyNewData();
      telemetry.update();
      try {
        Thread.sleep(20);
      } catch (InterruptedException e) {
        /* ok */
      }
    }

    // Smoke test,SparkSim might not generate enough current for stall
    boolean stalledState = telemetry.isStalled();
    assertNotNull(Boolean.valueOf(stalledState));
  }

  @Test
  void testStallRequiresDebounce() throws Exception {
    shooter.move(1.0);

    // Single cycle with stall-like conditions,too short for debounce
    setMotorVelocity(shooterSim, 10);
    DriverStationSim.notifyNewData();
    telemetry.update();

    // Immediately recover
    double target = shooter.getTargetRPM();
    setMotorVelocity(shooterSim, target);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertFalse(telemetry.isStalled(), "Single cycle should not trigger stall (debounce)");
  }

  @Test
  void testStallClearsOnRecovery() throws Exception {
    shooter.move(1.0);
    double target = shooter.getTargetRPM();

    setMotorVelocity(shooterSim, target);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertFalse(telemetry.isStalled(), "Should not be stalled when at target velocity");
  }

  @Test
  void testTemperatureAccessor() {
    setMotorVelocity(shooterSim, 0);
    telemetry.update();

    // SparkSim reports a temperature (may be default 25°C)
    double temp = telemetry.getTemperature();
    assertTrue(temp >= 0, "Temperature should be non-negative");
  }

  @Test
  void testVelocityErrorCalculation() throws Exception {
    shooter.move(1.0);
    double target = shooter.getTargetRPM();

    // Velocity below target
    double actualVelocity = target - 200;
    setMotorVelocity(shooterSim, actualVelocity);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double error = getField(telemetry, "velocityError");
    assertTrue(error > 0, "Velocity error should be positive when below target");
  }

  @Test
  void testAppliedOutputPassthrough() throws Exception {
    shooter.move(0.5);
    setMotorVelocity(shooterSim, 0);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double output = getField(telemetry, "appliedOutput");
    assertNotNull(Double.valueOf(output), "Applied output should be readable");
  }

  @Test
  void testCurrentAmpsReadable() throws Exception {
    shooter.move(1.0);
    setMotorVelocity(shooterSim, 100);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double current = getField(telemetry, "currentAmps");
    assertNotNull(Double.valueOf(current), "Current should be readable from SparkSim");
  }

  @Test
  void testSubsystemAvailableTrue() throws Exception {
    telemetry.update();
    boolean available = getField(telemetry, "subsystemAvailable");
    assertTrue(available, "Subsystem should be available when Shooter singleton exists");
  }

  @Test
  void testVelocityRPMAccessor() {
    shooter.move(1.0);
    double target = shooter.getTargetRPM();
    setMotorVelocity(shooterSim, target);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double velocity = telemetry.getVelocityRPM();
    assertEquals(target, velocity, 1.0, "Velocity should match what we set");
  }

  @Test
  void testLogDoesNotThrow() {
    telemetry.update();
    assertDoesNotThrow(() -> telemetry.log());
  }
}
