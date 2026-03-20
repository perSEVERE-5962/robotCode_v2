package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.util.ScoringReadiness;
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
    // ScoringReadiness is a singleton, needs reset between tests to avoid leaking state
    ScoringReadiness.getInstance().reset();
  }

  private void setDependencyField(Object obj, String fieldName, Object value) throws Exception {
    java.lang.reflect.Field f = obj.getClass().getDeclaredField(fieldName);
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

    // hub active state now lives in ScoringReadiness
    assertTrue(
        ScoringReadiness.getInstance().isHubActive(), "Hub should be active in non-teleop mode");
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

    telemetry.update();
    assertTrue(telemetry.isReadyToShoot(), "Should be ready");
    assertTrue(
        telemetry.isReadyStateChanged(), "State should have changed from default false to true");

    telemetry.update();
    assertFalse(telemetry.isReadyStateChanged(), "No change when still ready");
  }

  @Test
  void testReadyLostReasonCapturesFailingCondition() throws Exception {
    setDependencyField(shooterTelemetry, "atSpeed", true);
    setDependencyField(indexerTelemetry, "jamDetected", false);
    setDependencyField(visionTelemetry, "lockedOnTarget", true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
    telemetry.update();
    assertTrue(telemetry.isReadyToShoot());

    setDependencyField(visionTelemetry, "lockedOnTarget", false);

    // run enough cycles to expire the debounce window
    for (int i = 0; i < 50; i++) {
      SimHooks.stepTiming(0.02);
      telemetry.update();
    }

    assertFalse(telemetry.isReadyToShoot(), "Should lose ready after debounce");
    String reason = telemetry.getReadyLostReason();
    assertTrue(reason.contains("Vision"), "Lost reason should mention Vision, got: " + reason);
  }

  @Test
  void testReadyLostReasonCapturesMultipleFailures() throws Exception {
    setDependencyField(shooterTelemetry, "atSpeed", true);
    setDependencyField(indexerTelemetry, "jamDetected", false);
    setDependencyField(visionTelemetry, "lockedOnTarget", true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
    telemetry.update();

    setDependencyField(shooterTelemetry, "atSpeed", false);
    setDependencyField(visionTelemetry, "lockedOnTarget", false);

    // first update records falseSince timestamp in both debouncers
    telemetry.update();
    // step past both debounce windows (shooter 0.08s, vision 0.10s)
    SimHooks.stepTiming(0.15);
    // this update sees both expired, captures both in the reason
    telemetry.update();

    assertFalse(telemetry.isReadyToShoot());
    String reason = telemetry.getReadyLostReason();
    assertTrue(reason.contains("Shooter"), "Should mention Shooter, got: " + reason);
    assertTrue(reason.contains("Vision"), "Should mention Vision, got: " + reason);
  }

  @Test
  void testReadyLostReasonClearsWhenReadyRegained() throws Exception {
    setDependencyField(shooterTelemetry, "atSpeed", true);
    setDependencyField(indexerTelemetry, "jamDetected", false);
    setDependencyField(visionTelemetry, "lockedOnTarget", true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
    telemetry.update();

    setDependencyField(visionTelemetry, "lockedOnTarget", false);
    for (int i = 0; i < 50; i++) {
      SimHooks.stepTiming(0.02);
      telemetry.update();
    }
    assertFalse(telemetry.isReadyToShoot());
    assertFalse(telemetry.getReadyLostReason().isEmpty(), "Should have a reason");

    setDependencyField(visionTelemetry, "lockedOnTarget", true);
    telemetry.update();
    assertTrue(telemetry.isReadyToShoot());
    assertEquals("", telemetry.getReadyLostReason(), "Reason should clear when ready regained");
  }
}
