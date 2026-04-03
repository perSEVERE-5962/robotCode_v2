package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants;
//import frc.robot.Constants.ShooterConstants;
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
    shooter.moveToVelocityWithPID(0); // Reset targetRPM to 0 (singleton persists across tests)
    setMotorVelocity(shooterSim, 0);
  }

  @Test
  void testSpinUpTimingTracked() throws Exception {
    shooter.moveToVelocityWithPID(Shooter.getTunableTargetRPM());

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
    shooter.moveToVelocityWithPID(Shooter.getTunableTargetRPM());
    double target = shooter.getTargetRPM();
    double tolerance = Shooter.getToleranceRPM();

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
    shooter.moveToVelocityWithPID(Shooter.getTunableTargetRPM());
    double target = shooter.getTargetRPM();

    setMotorVelocity(shooterSim, target);
    for (int i = 0; i < 3; i++) {
      DriverStationSim.notifyNewData();
      telemetry.update();
    }

    assertEquals(0, telemetry.getTotalShots(), "No shots yet");
    assertTrue(telemetry.isAtSpeed(), "Should be at speed before shot test");

    double dippedVelocity = target - Shooter.getShotDropThreshold() - 100;
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
    shooter.moveToVelocityWithPID(Shooter.getTunableTargetRPM());
    double target = shooter.getTargetRPM();
    double dipped = target - Shooter.getShotDropThreshold() - 100;

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
    shooter.moveToVelocityWithPID(Shooter.getTunableTargetRPM());
    double target = shooter.getTargetRPM();
    double dipped = target - Shooter.getShotDropThreshold() - 100;

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
    shooter.moveToVelocityWithPID(Shooter.getTunableTargetRPM());
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

    shooter.moveToVelocityWithPID(Shooter.getTunableTargetRPM());
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

  private void setField(Object obj, String fieldName, Object value) throws Exception {
    java.lang.reflect.Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    f.set(obj, value);
  }
}
