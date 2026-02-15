package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.spark.SparkSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.IntakeActuator;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.MethodName.class)
class IntakeActuatorTelemetryTest {

  private static SparkSim actuatorSim;
  private IntakeActuatorTelemetry telemetry;
  private IntakeActuator intakeActuator;

  @BeforeAll
  static void initHardware() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
    RoboRioSim.setVInVoltage(12.5);

    IntakeActuator ia = IntakeActuator.getInstance();
    actuatorSim = new SparkSim(ia.getMotor(), DCMotor.getNEO(1));
  }

  @BeforeEach
  void setUp() {
    SafeLog.logAndReset();
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();

    telemetry = new IntakeActuatorTelemetry();
    intakeActuator = IntakeActuator.getInstance();
    actuatorSim.getRelativeEncoderSim().setPosition(0);
  }

  @AfterAll
  static void tearDown() {
    closeMotor("frc.robot.subsystems.IntakeActuator");
    resetSingleton("frc.robot.subsystems.IntakeActuator");
  }

  private static void closeMotor(String className) {
    try {
      Class<?> clazz = Class.forName(className);
      java.lang.reflect.Field f = clazz.getDeclaredField("instance");
      f.setAccessible(true);
      Object instance = f.get(null);
      if (instance != null) {
        java.lang.reflect.Method getMotor = clazz.getMethod("getMotor");
        Object motor = getMotor.invoke(instance);
        if (motor instanceof AutoCloseable) {
          ((AutoCloseable) motor).close();
        }
      }
    } catch (Exception e) {
    }
  }

  private static void resetSingleton(String className) {
    try {
      Class<?> clazz = Class.forName(className);
      java.lang.reflect.Field f = clazz.getDeclaredField("instance");
      f.setAccessible(true);
      f.set(null, null);
    } catch (Exception e) {
    }
  }

  @SuppressWarnings("unchecked")
  private <T> T getField(Object obj, String fieldName) throws Exception {
    java.lang.reflect.Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    return (T) f.get(obj);
  }

  @Test
  void testGetNameReturnsIntakeActuator() {
    assertEquals("IntakeActuator", telemetry.getName());
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
  void testDefaultsWhenStopped() throws Exception {
    actuatorSim.getRelativeEncoderSim().setPosition(0);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double position = getField(telemetry, "positionRotations");
    assertEquals(0, position, 0.01);

    double appliedOutput = getField(telemetry, "appliedOutput");
    assertEquals(0, appliedOutput, 0.01);
  }

  @Test
  void testSubsystemAvailableTrue() throws Exception {
    telemetry.update();
    boolean available = getField(telemetry, "subsystemAvailable");
    assertTrue(available, "Subsystem should be available when IntakeActuator singleton exists");
  }

  @Test
  void testPositionReading() throws Exception {
    actuatorSim.getRelativeEncoderSim().setPosition(2.5);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double position = getField(telemetry, "positionRotations");
    assertEquals(2.5, position, 0.01, "Position should match what we set");
  }

  @Test
  void testAtTargetWhenNearTarget() throws Exception {
    intakeActuator.moveToPositionWithPID(5.0);
    actuatorSim.getRelativeEncoderSim().setPosition(5.0);
    DriverStationSim.notifyNewData();
    telemetry.update();

    boolean atTarget = getField(telemetry, "atTarget");
    assertTrue(atTarget, "Should be at target when position matches target");
  }

  @Test
  void testNotAtTargetWhenFar() throws Exception {
    intakeActuator.moveToPositionWithPID(5.0);
    actuatorSim.getRelativeEncoderSim().setPosition(0);
    DriverStationSim.notifyNewData();
    telemetry.update();

    boolean atTarget = getField(telemetry, "atTarget");
    assertFalse(atTarget, "Should not be at target when far from target");
  }

  @Test
  void testTargetPositionTracked() throws Exception {
    intakeActuator.moveToPositionWithPID(3.14);
    telemetry.update();

    double target = getField(telemetry, "targetPosition");
    assertEquals(3.14, target, 0.001, "Target position should be tracked");
  }

  @Test
  void testTemperatureAccessor() {
    telemetry.update();
    double temp = telemetry.getTemperature();
    assertTrue(temp >= 0, "Temperature should be non-negative");
  }

  @Test
  void testDeviceConnectedTrue() throws Exception {
    telemetry.update();
    boolean connected = getField(telemetry, "deviceConnected");
    assertTrue(connected, "Device should be connected in simulation");
  }

  @Test
  void testDeviceFaultsReadable() throws Exception {
    telemetry.update();
    int faults = getField(telemetry, "deviceFaultsRaw");
    assertNotNull(Integer.valueOf(faults), "Faults should be readable");
  }
}
