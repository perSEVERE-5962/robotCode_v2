package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AgitatorTelemetryTest {

  private AgitatorTelemetry telemetry;

  @BeforeAll
  static void initHardware() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
    RoboRioSim.setVInVoltage(12.5);
  }

  @BeforeEach
  void setUp() {
    SafeLog.logAndReset();
    telemetry = new AgitatorTelemetry();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterAll
  static void tearDown() {
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
  void testStallDefaultFalse() throws Exception {
    telemetry.update();
    assertFalse(telemetry.isStalled(), "Stall should default to false");
  }

  @Test
  void testJamProtectionStateDefault() throws Exception {
    telemetry.update();

    String state = getField("jamProtectionState");
    assertEquals("MONITORING", state, "JamProtection should default to MONITORING");
  }
}
