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
    driverSim.setLeftBumperButton(false);
    driverSim.setRightBumperButton(false);
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
    operatorSim.setLeftBumperButton(false);
    operatorSim.setRightBumperButton(false);
    operatorSim.setStartButton(false);
    operatorSim.setBackButton(false);

    DriverStationSim.notifyNewData();
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

    DriverStationSim.notifyNewData();

    telemetry.update();

    assertFalse(telemetry.isDriverActive(), "Driver should be inactive with zero input");
  }

  @Test
  void testDriverActiveResetsIdleTime() {
    telemetry.setControllers(driver, operator);

    DriverStationSim.notifyNewData();
    telemetry.update();
    advanceTime();
    telemetry.update();
    assertTrue(telemetry.getDriverIdleTimeMs() > 0, "Should have idle time before activating");

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

    assertEquals(
        1,
        (int) packMethod.invoke(telemetry, true, false, false, false, false, false, false, false),
        "A button should be bit 0");

    assertEquals(
        255,
        (int) packMethod.invoke(telemetry, true, true, true, true, true, true, true, true),
        "All buttons should be 255");

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
}
