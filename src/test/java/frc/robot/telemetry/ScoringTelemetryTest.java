package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.lang.reflect.Field;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

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
    Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    f.set(obj, value);
  }

  @Test
  void testReadyToShootAllConditionsMet() throws Exception {
    setDependencyField(shooterTelemetry, "atSpeed", true);
    setDependencyField(indexerTelemetry, "jamDetected", false);
    setDependencyField(visionTelemetry, "lockedOnTarget", true);

    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();

    telemetry.update();
    assertTrue(telemetry.isReadyToShoot(), "All conditions met, should be ready");
  }

  @Test
  void testReadyToShootFailsWithoutShooter() throws Exception {
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
  void testHubActiveInNonTeleopMode() throws Exception {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    telemetry.update();

    boolean hubActive = getField(telemetry, "hubActive");
    assertTrue(hubActive, "Hub should be active in non-teleop mode");
  }

  @Test
  void testPreviousStateTracksLastCycleValues() throws Exception {
    // Cycle 1: shooter not ready, vision locked
    setDependencyField(shooterTelemetry, "atSpeed", false);
    setDependencyField(visionTelemetry, "lockedOnTarget", true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
    telemetry.update();

    // Cycle 2: shooter now ready
    setDependencyField(shooterTelemetry, "atSpeed", true);
    telemetry.update();

    // Previous should reflect cycle 1 values
    boolean prevShooterReady = getField(telemetry, "previousShooterReady");
    boolean prevVisionLocked = getField(telemetry, "previousVisionLocked");
    assertFalse(prevShooterReady, "Previous shooter should be false from cycle 1");
    assertTrue(prevVisionLocked, "Previous vision should be true from cycle 1");
  }

  @Test
  void testReadyStateChangedDetectsTransition() throws Exception {
    setDependencyField(shooterTelemetry, "atSpeed", true);
    setDependencyField(indexerTelemetry, "jamDetected", false);
    setDependencyField(visionTelemetry, "lockedOnTarget", true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();

    // Cycle 1: not ready (first cycle, previous is default false)
    // Actually first cycle readyToShoot becomes true immediately
    telemetry.update();
    assertTrue(telemetry.isReadyToShoot(), "Should be ready");
    assertTrue(
        telemetry.isReadyStateChanged(), "State should have changed from default false to true");

    // Cycle 2: still ready, no change
    telemetry.update();
    assertFalse(telemetry.isReadyStateChanged(), "No change when still ready");
  }

  @Test
  void testReadyLostReasonCapturesFailingCondition() throws Exception {
    // Get to ready state
    setDependencyField(shooterTelemetry, "atSpeed", true);
    setDependencyField(indexerTelemetry, "jamDetected", false);
    setDependencyField(visionTelemetry, "lockedOnTarget", true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
    telemetry.update();
    assertTrue(telemetry.isReadyToShoot());

    // Break vision lock
    setDependencyField(visionTelemetry, "lockedOnTarget", false);

    // Run enough cycles to expire the debounce window
    for (int i = 0; i < 50; i++) {
      try {
        Thread.sleep(20);
      } catch (InterruptedException e) {
        /* ok */
      }
      telemetry.update();
    }

    assertFalse(telemetry.isReadyToShoot(), "Should lose ready after debounce");
    String reason = telemetry.getReadyLostReason();
    assertTrue(reason.contains("Vision"), "Lost reason should mention Vision, got: " + reason);
  }

  @Test
  void testReadyLostReasonCapturesMultipleFailures() throws Exception {
    // Get to ready state
    setDependencyField(shooterTelemetry, "atSpeed", true);
    setDependencyField(indexerTelemetry, "jamDetected", false);
    setDependencyField(visionTelemetry, "lockedOnTarget", true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
    telemetry.update();

    // Break shooter AND vision simultaneously
    setDependencyField(shooterTelemetry, "atSpeed", false);
    setDependencyField(visionTelemetry, "lockedOnTarget", false);

    // Run enough cycles to expire debounce
    for (int i = 0; i < 50; i++) {
      try {
        Thread.sleep(20);
      } catch (InterruptedException e) {
        /* ok */
      }
      telemetry.update();
    }

    assertFalse(telemetry.isReadyToShoot());
    String reason = telemetry.getReadyLostReason();
    assertTrue(reason.contains("Shooter"), "Should mention Shooter, got: " + reason);
    assertTrue(reason.contains("Vision"), "Should mention Vision, got: " + reason);
  }

  @Test
  void testReadyLostReasonClearsWhenReadyRegained() throws Exception {
    // Get to ready state
    setDependencyField(shooterTelemetry, "atSpeed", true);
    setDependencyField(indexerTelemetry, "jamDetected", false);
    setDependencyField(visionTelemetry, "lockedOnTarget", true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
    telemetry.update();

    // Break and expire debounce
    setDependencyField(visionTelemetry, "lockedOnTarget", false);
    for (int i = 0; i < 50; i++) {
      try {
        Thread.sleep(20);
      } catch (InterruptedException e) {
        /* ok */
      }
      telemetry.update();
    }
    assertFalse(telemetry.isReadyToShoot());
    assertFalse(telemetry.getReadyLostReason().isEmpty(), "Should have a reason");

    // Restore and regain ready
    setDependencyField(visionTelemetry, "lockedOnTarget", true);
    telemetry.update();
    assertTrue(telemetry.isReadyToShoot());
    assertEquals("", telemetry.getReadyLostReason(), "Reason should clear when ready regained");
  }
}
