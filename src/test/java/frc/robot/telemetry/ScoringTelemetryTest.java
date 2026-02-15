package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.MethodName.class)
class ScoringTelemetryTest extends TelemetryTestBase {

  private ShooterTelemetry shooterTelemetry;
  private IndexerTelemetry indexerTelemetry;
  private VisionTelemetry visionTelemetry;
  private ScoringTelemetry telemetry;

  @BeforeEach
  void setUp() {
    shooterTelemetry = new ShooterTelemetry();
    indexerTelemetry = new IndexerTelemetry();
    visionTelemetry = new VisionTelemetry();
    telemetry = new ScoringTelemetry(shooterTelemetry, indexerTelemetry, visionTelemetry);
  }

  private void setDependencyField(Object obj, String fieldName, Object value) throws Exception {
    java.lang.reflect.Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    f.set(obj, value);
  }

  @Test
  void testGetNameReturnsScoring() {
    assertEquals("Scoring", telemetry.getName());
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
  void testNullDependenciesSetsDefaults() throws Exception {
    ScoringTelemetry nullTelemetry = new ScoringTelemetry(null, null, null);
    nullTelemetry.update();

    boolean available = getField(nullTelemetry, "scoringAvailable");
    assertFalse(available);
    assertFalse(nullTelemetry.isReadyToShoot());
  }

  @Test
  void testScoringAvailableWithValidDeps() throws Exception {
    telemetry.update();
    boolean available = getField(telemetry, "scoringAvailable");
    assertTrue(available, "Should be available when all dependencies provided");
  }

  @Test
  void testReadyToShootDefaultFalse() {
    telemetry.update();
    assertFalse(telemetry.isReadyToShoot(), "Not ready,shooter not at speed, vision not locked");
  }

  @Test
  void testIndexerClearByDefault() throws Exception {
    telemetry.update();
    boolean clear = getField(telemetry, "indexerClear");
    assertTrue(clear, "Indexer should be clear when no jam detected");
  }

  @Test
  void testHasBallAlwaysTrue() throws Exception {
    telemetry.update();
    boolean hasBall = getField(telemetry, "hasBall");
    assertTrue(hasBall, "HasBall is stubbed to always true");
  }

  @Test
  void testReadyToShootAllConditionsMet() throws Exception {
    // Set all dependency states to satisfy composite
    setDependencyField(shooterTelemetry, "atSpeed", true);
    setDependencyField(indexerTelemetry, "jamDetected", false);
    setDependencyField(visionTelemetry, "lockedOnTarget", true);

    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();

    telemetry.update();

    boolean shooterReady = getField(telemetry, "shooterReady");
    boolean indexerClear = getField(telemetry, "indexerClear");
    boolean visionLocked = getField(telemetry, "visionLocked");
    assertTrue(shooterReady);
    assertTrue(indexerClear);
    assertTrue(visionLocked);
    assertTrue(telemetry.isReadyToShoot(), "All conditions met,should be ready");
  }

  @Test
  void testReadyToShootFailsWithoutShooter() throws Exception {
    // Vision locked but shooter not at speed
    setDependencyField(visionTelemetry, "lockedOnTarget", true);

    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();

    telemetry.update();
    assertFalse(telemetry.isReadyToShoot(), "Not ready without shooter at speed");
  }

  @Test
  void testReadyToShootFailsWithoutVision() throws Exception {
    // Shooter ready but vision not locked
    setDependencyField(shooterTelemetry, "atSpeed", true);

    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();

    telemetry.update();
    assertFalse(telemetry.isReadyToShoot(), "Not ready without vision lock");
  }

  @Test
  void testReadyToShootFailsWithJam() throws Exception {
    // All good except indexer jammed
    setDependencyField(shooterTelemetry, "atSpeed", true);
    setDependencyField(indexerTelemetry, "jamDetected", true);
    setDependencyField(visionTelemetry, "lockedOnTarget", true);

    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();

    telemetry.update();
    assertFalse(telemetry.isReadyToShoot(), "Not ready with indexer jammed");
  }

  @Test
  void testTimeSinceReadyZeroWhenNotReady() throws Exception {
    telemetry.update();
    double time = getField(telemetry, "timeSinceReadyMs");
    assertEquals(0, time, 0.01);
  }

  @Test
  void testHubActiveInNonTeleopMode() throws Exception {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    telemetry.update();

    boolean hubActive = getField(telemetry, "hubActive");
    assertTrue(hubActive, "Hub should be active in non-teleop mode");
  }
}
