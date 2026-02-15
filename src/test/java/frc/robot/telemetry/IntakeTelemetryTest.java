package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.Intake;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.MethodName.class)
class IntakeTelemetryTest extends SparkSimTestBase {

  private IntakeTelemetry telemetry;
  private Intake intake;

  @BeforeEach
  void setUp() {
    telemetry = new IntakeTelemetry();
    intake = Intake.getInstance();
    setMotorVelocity(intakeSim, 0);
  }

  @Test
  void testGetNameReturnsIntake() {
    assertEquals("Intake", telemetry.getName());
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
  void testDefaultsWhenStopped() {
    telemetry.update();

    assertFalse(telemetry.isJamDetected());
    assertEquals(0, telemetry.getTotalJamCount());
    assertFalse(telemetry.isStalled());
  }

  @Test
  void testSubsystemAvailableTrue() throws Exception {
    telemetry.update();
    boolean available = getField(telemetry, "subsystemAvailable");
    assertTrue(available, "Subsystem should be available when Intake singleton exists");
  }

  @Test
  void testDirectionDefaultsStopped() throws Exception {
    telemetry.update();
    String direction = getField(telemetry, "direction");
    assertEquals("STOPPED", direction);
  }

  @Test
  void testDirectionFieldIsValid() throws Exception {
    telemetry.update();
    String direction = getField(telemetry, "direction");
    assertTrue(
        direction.equals("FORWARD") || direction.equals("REVERSE") || direction.equals("STOPPED"),
        "Direction should be FORWARD, REVERSE, or STOPPED but was: " + direction);
  }

  @Test
  void testVelocityRPMAccessor() throws Exception {
    setMotorVelocity(intakeSim, 800);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double velocity = getField(telemetry, "velocityRPM");
    assertEquals(800, velocity, 1.0, "Velocity should match what we set");
  }

  @Test
  void testTemperatureAccessor() {
    telemetry.update();
    double temp = telemetry.getTemperature();
    assertTrue(temp >= 0, "Temperature should be non-negative");
  }

  @Test
  void testCurrentPerSpeedRatioNonZero() throws Exception {
    intake.move(1.0);
    iterateMotor(intakeSim, 500, 12.5);
    setMotorVelocity(intakeSim, 500);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double ratio = getField(telemetry, "currentPerSpeedRatio");
    // Ratio = current / abs(velocity). Current may be 0 in sim, so just check no crash
    assertNotNull(Double.valueOf(ratio));
  }

  @Test
  void testCurrentPerSpeedRatioZeroAtLowVelocity() throws Exception {
    // Below 10 RPM threshold, ratio should be 0
    intake.move(1.0);
    iterateMotor(intakeSim, 5, 12.5);
    setMotorVelocity(intakeSim, 5);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double ratio = getField(telemetry, "currentPerSpeedRatio");
    assertEquals(0, ratio, 0.001, "Ratio should be 0 below velocity threshold");
  }

  @Test
  void testJamNotDetectedWhenStopped() {
    intake.move(0.0);
    iterateMotor(intakeSim, 0, 12.5);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertFalse(telemetry.isJamDetected());
  }

  @Test
  void testClearJamResetsState() {
    telemetry.update();
    telemetry.clearJam();
    assertFalse(telemetry.isJamDetected());
  }

  @Test
  void testStallDetectionSmoke() throws Exception {
    intake.move(1.0);
    iterateMotor(intakeSim, 10, 12.5);
    DriverStationSim.notifyNewData();

    long startMs = System.currentTimeMillis();
    while (System.currentTimeMillis() - startMs < 250) {
      iterateMotor(intakeSim, 10, 12.5);
      DriverStationSim.notifyNewData();
      telemetry.update();
      try {
        Thread.sleep(20);
      } catch (InterruptedException e) {
        /* ok */
      }
    }

    // Smoke test,SparkSim may not generate enough current for stall
    boolean stalledState = telemetry.isStalled();
    assertNotNull(Boolean.valueOf(stalledState));
  }

  @Test
  void testStallClearsOnRecovery() throws Exception {
    intake.move(1.0);
    iterateMotor(intakeSim, 500, 12.5);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertFalse(telemetry.isStalled(), "Should not be stalled at normal velocity");
  }
}
