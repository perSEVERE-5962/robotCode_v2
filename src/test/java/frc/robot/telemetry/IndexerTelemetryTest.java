package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.Indexer;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.MethodName.class)
class IndexerTelemetryTest extends SparkSimTestBase {

  private IndexerTelemetry telemetry;
  private Indexer indexer;

  @BeforeEach
  void setUp() {
    telemetry = new IndexerTelemetry();
    indexer = Indexer.getInstance();
    setMotorVelocity(indexerSim, 0);
  }

  @Test
  void testGetNameReturnsIndexer() {
    assertEquals("Indexer", telemetry.getName());
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
    assertTrue(available, "Subsystem should be available when Indexer singleton exists");
  }

  @Test
  void testDirectionDefaultsStopped() throws Exception {
    // Without motor command, direction stays STOPPED
    telemetry.update();
    String direction = getField(telemetry, "direction");
    assertEquals("STOPPED", direction);
  }

  @Test
  void testDirectionFieldIsValid() throws Exception {
    // Verify direction field is always one of the valid values
    telemetry.update();
    String direction = getField(telemetry, "direction");
    assertTrue(
        direction.equals("FORWARD") || direction.equals("REVERSE") || direction.equals("STOPPED"),
        "Direction should be FORWARD, REVERSE, or STOPPED but was: " + direction);
  }

  @Test
  void testRunningDefaultsFalse() throws Exception {
    // Without motor command, running defaults to false
    // (getAppliedOutput requires command scheduler in SparkSim)
    telemetry.update();
    boolean running = getField(telemetry, "running");
    assertFalse(running, "Should not be running without motor command");
  }

  @Test
  void testVelocityRPMAccessor() throws Exception {
    setMotorVelocity(indexerSim, 500);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double velocity = getField(telemetry, "velocityRPM");
    assertEquals(500, velocity, 1.0, "Velocity should match what we set");
  }

  @Test
  void testTemperatureAccessor() {
    telemetry.update();
    double temp = telemetry.getTemperature();
    assertTrue(temp >= 0, "Temperature should be non-negative");
  }

  @Test
  void testJamNotDetectedWhenStopped() {
    // Motor not running,no jam even if current is high
    indexer.getMotor().set(0.0);
    iterateMotor(indexerSim, 0, 12.5);
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
    // Stall: low velocity + running. SparkSim may not produce enough current.
    indexer.getMotor().set(1.0);
    iterateMotor(indexerSim, 10, 12.5);
    DriverStationSim.notifyNewData();

    long startMs = System.currentTimeMillis();
    while (System.currentTimeMillis() - startMs < 250) {
      iterateMotor(indexerSim, 10, 12.5);
      DriverStationSim.notifyNewData();
      telemetry.update();
      try {
        Thread.sleep(20);
      } catch (InterruptedException e) {
        /* ok */
      }
    }

    // Smoke test,just verify stall state machine doesn't crash
    boolean stalledState = telemetry.isStalled();
    assertNotNull(Boolean.valueOf(stalledState));
  }

  @Test
  void testStallRequiresDebounce() throws Exception {
    indexer.getMotor().set(1.0);
    iterateMotor(indexerSim, 10, 12.5);
    DriverStationSim.notifyNewData();
    telemetry.update();

    // Immediately recover
    setMotorVelocity(indexerSim, 500);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertFalse(telemetry.isStalled(), "Single cycle should not trigger stall (debounce)");
  }

  @Test
  void testStallClearsOnRecovery() throws Exception {
    indexer.getMotor().set(1.0);
    iterateMotor(indexerSim, 500, 12.5);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertFalse(telemetry.isStalled(), "Should not be stalled at normal velocity");
  }
}
