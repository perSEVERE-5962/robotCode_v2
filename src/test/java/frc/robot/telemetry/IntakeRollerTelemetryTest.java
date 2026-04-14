package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.IntakeRoller;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.MethodName.class)
class IntakeRollerTelemetryTest extends SparkSimTestBase {

  private IntakeRollerTelemetry telemetry;
  private IntakeRoller intake;

  @BeforeEach
  void setUp() {
    telemetry = new IntakeRollerTelemetry();
    intake = IntakeRoller.getInstance();
    intake.move(0);
    setMotorVelocity(intakeRollerSim, 0);
  }

  @Test
  void testDefaultsWhenStopped() throws Exception {
    telemetry.update();

    assertFalse(telemetry.isStalled());
    assertFalse(telemetry.isJamDetected());
    assertEquals(0, telemetry.getTotalJamCount());
    String direction = getField(telemetry, "direction");
    assertEquals("STOPPED", direction);
  }

  @Test
  void testVelocityReading() throws Exception {
    setMotorVelocity(intakeRollerSim, 3000);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double velocity = getField(telemetry, "velocityRPM");
    assertEquals(3000, velocity, 1.0, "Velocity should match what we set");
  }

  @Test
  void testJamNotTriggeredAtNormalVelocity() {
    intake.move(1.0);
    setMotorVelocity(intakeRollerSim, 500);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertFalse(telemetry.isJamDetected(), "Jam should not trigger at normal velocity");
  }

  @Test
  void testJamRequiresRunning() throws Exception {
    // High current but not running (motor not commanded)
    setMotorVelocity(intakeRollerSim, 0);
    DriverStationSim.notifyNewData();
    telemetry.update();

    setField(telemetry, "currentAmps", 30.0);
    telemetry.update();

    assertFalse(telemetry.isJamDetected(), "Jam should not trigger when not running");
  }

  @Test
  void testJamDefaultsToFalse() {
    telemetry.update();
    assertFalse(telemetry.isJamDetected());
  }

  @Test
  void testStallClearsWhenConditionClears() throws Exception {
    setField(telemetry, "stalled", true);
    setField(telemetry, "inStallCondition", true);
    setField(telemetry, "stallStartTime", Timer.getFPGATimestamp());

    intake.move(1.0);
    setMotorVelocity(intakeRollerSim, 500);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertFalse(telemetry.isStalled(), "Stall should clear when velocity recovers");
  }

  @Test
  void testStallNotTriggeredDuringStartup() throws Exception {
    intake.move(1.0);
    setMotorVelocity(intakeRollerSim, 0);
    DriverStationSim.notifyNewData();

    telemetry.update();

    setField(telemetry, "currentAmps", 25.0);
    setField(telemetry, "velocityRPM", 5.0);
    setField(telemetry, "running", true);

    for (int i = 0; i < 15; i++) {
      telemetry.update();
    }

    assertFalse(telemetry.isStalled(), "Should not trigger stall during startup ignore window");
  }

  @Test
  void testCurrentPerSpeedRatioPositiveWhenRunning() throws Exception {
    intake.move(1.0);
    setMotorVelocity(intakeRollerSim, 500);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double ratio = getField(telemetry, "currentPerSpeedRatio");
    assertTrue(ratio >= 0, "Current/speed ratio should be non-negative when running");
  }

  @Test
  void testCurrentPerSpeedRatioZeroWhenStopped() throws Exception {
    setMotorVelocity(intakeRollerSim, 0);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double ratio = getField(telemetry, "currentPerSpeedRatio");
    assertEquals(0, ratio, 0.001, "Current/speed ratio should be 0 when velocity < 10 RPM");
  }

  private void setField(Object obj, String fieldName, Object value) throws Exception {
    java.lang.reflect.Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    f.set(obj, value);
  }
}
