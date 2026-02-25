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
  void testStallClearsWhenConditionClears() throws Exception {
    setField(telemetry, "stalled", true);
    setField(telemetry, "inStallCondition", true);
    setField(telemetry, "stallStartTime", edu.wpi.first.wpilibj.Timer.getFPGATimestamp());

    indexer.getMotor().set(1.0);
    setMotorVelocity(indexerSim, 500);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertFalse(telemetry.isStalled(), "Stall should clear when velocity recovers");
  }

  @Test
  void testStallNotTriggeredDuringStartup() throws Exception {
    indexer.getMotor().set(1.0);
    setMotorVelocity(indexerSim, 0);
    DriverStationSim.notifyNewData();

    telemetry.update();

    setField(telemetry, "currentAmps", 30.0);
    setField(telemetry, "velocityRPM", 5.0);
    setField(telemetry, "running", true);

    for (int i = 0; i < 15; i++) {
      telemetry.update();
    }

    assertFalse(telemetry.isStalled(), "Should not trigger stall during startup ignore window");
  }

  @Test
  void testClearJamResetsState() {
    telemetry.update();
    telemetry.clearJam();
    assertFalse(telemetry.isJamDetected());
  }

  @Test
  void testJamRequiresLowVelocity() throws Exception {
    indexer.getMotor().set(1.0);
    setMotorVelocity(indexerSim, 500);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertFalse(telemetry.isJamDetected(), "Jam should not trigger at normal velocity");
  }

  @Test
  void testJamClearsWhenConditionClears() throws Exception {
    setField(telemetry, "jamDetected", true);
    setField(telemetry, "inJamCondition", true);
    setField(telemetry, "jamConditionStartTime", edu.wpi.first.wpilibj.Timer.getFPGATimestamp());

    indexer.getMotor().set(1.0);
    setMotorVelocity(indexerSim, 500);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertFalse(telemetry.isJamDetected(), "Jam should clear when current drops");
  }

  @Test
  void testDirectionDefaultsStopped() throws Exception {
    telemetry.update();
    String direction = getField(telemetry, "direction");
    assertEquals("STOPPED", direction);
  }

  @Test
  void testVelocityRPMAccessor() throws Exception {
    setMotorVelocity(indexerSim, 500);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double velocity = getField(telemetry, "velocityRPM");
    assertEquals(500, velocity, 1.0, "Velocity should match what we set");
  }

  private void setField(Object obj, String fieldName, Object value) throws Exception {
    java.lang.reflect.Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    f.set(obj, value);
  }
}
