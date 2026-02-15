package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import java.lang.reflect.Method;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriverInputTelemetryTest extends TelemetryTestBase {

  private DriverInputTelemetry telemetry;
  private XboxController driver;
  private XboxController operator;
  private XboxControllerSim driverSim;
  private XboxControllerSim operatorSim;

  @BeforeEach
  void setUp() {
    telemetry = new DriverInputTelemetry();

    driver = new XboxController(0);
    operator = new XboxController(1);
    driverSim = new XboxControllerSim(driver);
    operatorSim = new XboxControllerSim(operator);

    // Initialize all axes to zero so the joystick is recognized
    zeroAllInputs();
  }

  private void zeroAllInputs() {
    driverSim.setLeftX(0.0);
    driverSim.setLeftY(0.0);
    driverSim.setRightX(0.0);
    driverSim.setRightY(0.0);
    driverSim.setLeftTriggerAxis(0.0);
    driverSim.setRightTriggerAxis(0.0);
    driverSim.setAButton(false);
    driverSim.setBButton(false);
    driverSim.setXButton(false);
    driverSim.setYButton(false);
    driverSim.setLeftBumper(false);
    driverSim.setRightBumper(false);
    driverSim.setStartButton(false);
    driverSim.setBackButton(false);

    operatorSim.setLeftX(0.0);
    operatorSim.setLeftY(0.0);
    operatorSim.setRightX(0.0);
    operatorSim.setRightY(0.0);
    operatorSim.setLeftTriggerAxis(0.0);
    operatorSim.setRightTriggerAxis(0.0);
    operatorSim.setAButton(false);
    operatorSim.setBButton(false);
    operatorSim.setXButton(false);
    operatorSim.setYButton(false);
    operatorSim.setLeftBumper(false);
    operatorSim.setRightBumper(false);
    operatorSim.setStartButton(false);
    operatorSim.setBackButton(false);

    DriverStationSim.notifyNewData();
  }

  @Test
  void testNoControllersDoesNotCrash() {
    DriverInputTelemetry bare = new DriverInputTelemetry();
    assertDoesNotThrow(
        () -> {
          bare.update();
          bare.log();
        });
  }

  @Test
  void testDriverInputMagnitude() {
    telemetry.setControllers(driver, operator);

    driverSim.setLeftX(0.5);
    driverSim.setLeftY(-0.8);
    DriverStationSim.notifyNewData();

    telemetry.update();

    assertTrue(telemetry.isDriverActive(), "Driver should be active with significant stick input");
  }

  @Test
  void testDriverInactiveWithZeroInput() {
    telemetry.setControllers(driver, operator);

    // All already zeroed in setUp
    DriverStationSim.notifyNewData();

    telemetry.update();

    assertFalse(telemetry.isDriverActive(), "Driver should be inactive with zero input");
  }

  @Test
  void testDriverIdleTracking() {
    telemetry.setControllers(driver, operator);

    DriverStationSim.notifyNewData();

    telemetry.update(); // First update initializes lastDriverInputTime
    advanceTime();
    telemetry.update(); // Second update should show idle time

    assertTrue(
        telemetry.getDriverIdleTimeMs() > 0, "Idle time should increase when driver is inactive");
  }

  @Test
  void testDriverActiveResetsIdleTime() {
    telemetry.setControllers(driver, operator);

    // Idle first
    DriverStationSim.notifyNewData();
    telemetry.update();
    advanceTime();
    telemetry.update();
    assertTrue(telemetry.getDriverIdleTimeMs() > 0, "Should have idle time before activating");

    // Now activate
    driverSim.setLeftX(0.5);
    DriverStationSim.notifyNewData();
    telemetry.update();
    assertEquals(
        0.0,
        telemetry.getDriverIdleTimeMs(),
        0.001,
        "Idle time should reset to 0 when driver becomes active");
  }

  @Test
  void testButtonPacking() throws Exception {
    Method packMethod =
        DriverInputTelemetry.class.getDeclaredMethod(
            "packButtons",
            boolean.class,
            boolean.class,
            boolean.class,
            boolean.class,
            boolean.class,
            boolean.class,
            boolean.class,
            boolean.class);
    packMethod.setAccessible(true);

    // A=1, B=2, X=4, Y=8, LB=16, RB=32, Start=64, Back=128
    assertEquals(
        1,
        (int) packMethod.invoke(telemetry, true, false, false, false, false, false, false, false),
        "A button should be bit 0");

    assertEquals(
        2,
        (int) packMethod.invoke(telemetry, false, true, false, false, false, false, false, false),
        "B button should be bit 1");

    assertEquals(
        4,
        (int) packMethod.invoke(telemetry, false, false, true, false, false, false, false, false),
        "X button should be bit 2");

    assertEquals(
        8,
        (int) packMethod.invoke(telemetry, false, false, false, true, false, false, false, false),
        "Y button should be bit 3");

    assertEquals(
        16,
        (int) packMethod.invoke(telemetry, false, false, false, false, true, false, false, false),
        "LB should be bit 4");

    assertEquals(
        32,
        (int) packMethod.invoke(telemetry, false, false, false, false, false, true, false, false),
        "RB should be bit 5");

    assertEquals(
        64,
        (int) packMethod.invoke(telemetry, false, false, false, false, false, false, true, false),
        "Start should be bit 6");

    assertEquals(
        128,
        (int) packMethod.invoke(telemetry, false, false, false, false, false, false, false, true),
        "Back should be bit 7");

    assertEquals(
        255,
        (int) packMethod.invoke(telemetry, true, true, true, true, true, true, true, true),
        "All buttons should be 255");

    // A + X + LB = 1 + 4 + 16 = 21
    assertEquals(
        21,
        (int) packMethod.invoke(telemetry, true, false, true, false, true, false, false, false),
        "A+X+LB should be 21");
  }

  @Test
  void testTriggerActivatesDriver() {
    telemetry.setControllers(driver, operator);

    driverSim.setLeftTriggerAxis(0.8);
    DriverStationSim.notifyNewData();

    telemetry.update();

    assertTrue(telemetry.isDriverActive(), "Driver should be active when trigger exceeds deadband");
  }

  @Test
  void testGetNameReturnsDriverInput() {
    assertEquals("DriverInput", telemetry.getName());
  }

  @Test
  void testUpdateAndLogWithControllersDoNotThrow() {
    telemetry.setControllers(driver, operator);
    DriverStationSim.notifyNewData();

    assertDoesNotThrow(
        () -> {
          telemetry.update();
          telemetry.log();
        });
  }
}
