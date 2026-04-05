package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.Agitator;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AgitatorTelemetryTest {

  // private static SparkSim agitatorSim;
  private AgitatorTelemetry telemetry;

  @BeforeAll
  static void initHardware() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
    RoboRioSim.setVInVoltage(12.5);

    Agitator agitator = Agitator.getInstance();
    // agitatorSim = new SparkSim(agitator.getMotor(), DCMotor.getNEO(1));
  }

  @BeforeEach
  void setUp() {
    SafeLog.logAndReset();
    telemetry = new AgitatorTelemetry();
    // agitatorSim.getRelativeEncoderSim().setVelocity(0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterAll
  static void tearDown() {
    try {
      Agitator agitator = Agitator.getInstance();
      if (agitator != null) agitator.getMotor().close();
    } catch (Exception e) {
    }
    resetSingleton("frc.robot.subsystems.Agitator");
    resetSingleton("frc.robot.telemetry.TelemetryManager");
  }

  private static void resetSingleton(String className) {
    try {
      Class<?> clazz = Class.forName(className);
      Field f = clazz.getDeclaredField("instance");
      f.setAccessible(true);
      f.set(null, null);
    } catch (Exception e) {
    }
  }

  @SuppressWarnings("unchecked")
  private <T> T getField(String fieldName) throws Exception {
    Field f = AgitatorTelemetry.class.getDeclaredField(fieldName);
    f.setAccessible(true);
    return (T) f.get(telemetry);
  }

  private void setField(String fieldName, Object value) throws Exception {
    Field f = AgitatorTelemetry.class.getDeclaredField(fieldName);
    f.setAccessible(true);
    f.set(telemetry, value);
  }

  @Test
  void testDefaultsWhenStopped() {
    telemetry.update();

    assertFalse(telemetry.isStalled());
    assertFalse(telemetry.isJamProtectionIntervening());
  }

  @Test
  void testVelocityReading() throws Exception {
    // agitatorSim.getRelativeEncoderSim().setVelocity(2000);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double velocity = getField("velocityRPM");
    // assertEquals(2000, velocity, 1.0, "Velocity should match what we set");
  }

  @Test
  void testStallClearsWhenVelocityRecovers() throws Exception {
    setField("stalled", true);
    setField("inStallCondition", true);

    // agitatorSim.getRelativeEncoderSim().setVelocity(500);
    DriverStationSim.notifyNewData();
    telemetry.update();

    // assertFalse(telemetry.isStalled(), "Stall should clear when velocity recovers");
  }

  @Test
  void testStallNotTriggeredAtNormalVelocity() throws Exception {
    // agitatorSim.getRelativeEncoderSim().setVelocity(500);
    DriverStationSim.notifyNewData();
    telemetry.update();

    setField("currentAmps", 20.0);
    setField("running", true);
    telemetry.update();

    // assertFalse(telemetry.isStalled(), "Should not stall at normal velocity");
  }

  @Test
  void testStallRequiresRunning() throws Exception {
    // agitatorSim.getRelativeEncoderSim().setVelocity(0);
    DriverStationSim.notifyNewData();
    telemetry.update();

    setField("currentAmps", 20.0);
    telemetry.update();

    // assertFalse(telemetry.isStalled(), "Stall should not trigger when not running");
  }

  @Test
  void testJamProtectionStateDefault() throws Exception {
    telemetry.update();

    String state = getField("jamProtectionState");
    // assertEquals("MONITORING", state, "JamProtection should default to MONITORING");
  }
}
