package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.spark.SparkSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.Hanger;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.MethodName.class)
class HangerTelemetryTest {

  private static SparkSim hangerSim;
  private HangerTelemetry telemetry;
  private Hanger hanger;

  @BeforeAll
  static void initHardware() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
    RoboRioSim.setVInVoltage(12.5);

    Hanger h = Hanger.getInstance();
    hangerSim = new SparkSim(h.getMotor(), DCMotor.getNEO(1));
  }

  @BeforeEach
  void setUp() {
    SafeLog.logAndReset();
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();

    telemetry = new HangerTelemetry();
    hanger = Hanger.getInstance();
    hangerSim.getRelativeEncoderSim().setPosition(0);
  }

  @AfterAll
  static void tearDown() {
    closeMotor("frc.robot.subsystems.Hanger");
    resetSingleton("frc.robot.subsystems.Hanger");
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
  void testGetNameReturnsHanger() {
    assertEquals("Hanger", telemetry.getName());
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
    hangerSim.getRelativeEncoderSim().setPosition(0);
    DriverStationSim.notifyNewData();
    telemetry.update();

    assertFalse((boolean) getField(telemetry, "isDeployed"));
    assertFalse((boolean) getField(telemetry, "isClimbing"));
    assertEquals(0.0, (double) getField(telemetry, "position"), 0.01);
  }

  @Test
  void testSubsystemAvailableTrue() throws Exception {
    telemetry.update();
    boolean available = getField(telemetry, "subsystemAvailable");
    assertTrue(available, "Subsystem should be available when Hanger singleton exists");
  }

  @Test
  void testPositionReading() throws Exception {
    hangerSim.getRelativeEncoderSim().setPosition(10.0);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double position = getField(telemetry, "position");
    assertEquals(10.0, position, 0.01, "Position should match what we set");
  }

  @Test
  void testPositionErrorCalculation() throws Exception {
    hanger.moveToPositionWithPID(50.0);
    hangerSim.getRelativeEncoderSim().setPosition(30.0);
    DriverStationSim.notifyNewData();
    telemetry.update();

    double error = getField(telemetry, "positionError");
    assertEquals(20.0, error, 0.01, "Error should be target - position");
  }

  @Test
  void testTargetPositionTracked() throws Exception {
    hanger.moveToPositionWithPID(42.0);
    telemetry.update();

    double target = getField(telemetry, "targetPosition");
    assertEquals(42.0, target, 0.001, "Target position should be tracked");
  }

  @Test
  void testControlModeDefaultsOpenLoop() throws Exception {
    // targetPosition=0 by default → OPEN_LOOP
    telemetry.update();
    String mode = getField(telemetry, "controlMode");
    assertEquals("OPEN_LOOP", mode);
  }

  @Test
  void testControlModePositionWhenTargeted() throws Exception {
    hanger.moveToPositionWithPID(5.0);
    telemetry.update();

    String mode = getField(telemetry, "controlMode");
    assertEquals("POSITION", mode);
  }

  @Test
  void testIsDeployedAtUpPosition() throws Exception {
    hangerSim.getRelativeEncoderSim().setPosition(MotorConstants.UP_HANGER_POS);
    DriverStationSim.notifyNewData();
    telemetry.update();

    boolean deployed = getField(telemetry, "isDeployed");
    assertTrue(deployed, "Should be deployed at UP_HANGER_POS");
  }

  @Test
  void testNotDeployedAtZero() throws Exception {
    hangerSim.getRelativeEncoderSim().setPosition(0);
    DriverStationSim.notifyNewData();
    telemetry.update();

    boolean deployed = getField(telemetry, "isDeployed");
    assertFalse(deployed, "Should not be deployed at position 0");
  }

  @Test
  void testIsClimbingWhenTargetedDown() throws Exception {
    hanger.moveToPositionWithPID(MotorConstants.DOWN_HANGER_POS);
    hangerSim.getRelativeEncoderSim().setPosition(MotorConstants.UP_HANGER_POS);
    DriverStationSim.notifyNewData();
    telemetry.update();

    boolean climbing = getField(telemetry, "isClimbing");
    assertTrue(climbing, "Should be climbing when target is DOWN and not at position");
  }

  @Test
  void testNotClimbingWhenAtPosition() throws Exception {
    hanger.moveToPositionWithPID(MotorConstants.DOWN_HANGER_POS);
    hangerSim.getRelativeEncoderSim().setPosition(MotorConstants.DOWN_HANGER_POS);
    DriverStationSim.notifyNewData();
    telemetry.update();

    boolean climbing = getField(telemetry, "isClimbing");
    assertFalse(climbing, "Should not be climbing when at target position");
  }

  @Test
  void testClimbProgressClamped() throws Exception {
    telemetry.update();
    double progress = getField(telemetry, "climbProgress");
    assertTrue(progress >= 0 && progress <= 1, "Progress should be 0-1 clamped");
  }

  @Test
  void testSoftLimitActiveField() throws Exception {
    hangerSim.getRelativeEncoderSim().setPosition(0);
    DriverStationSim.notifyNewData();
    telemetry.update();

    boolean softLimit = getField(telemetry, "softLimitActive");
    assertNotNull(Boolean.valueOf(softLimit), "Soft limit field should be readable");
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
