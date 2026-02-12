package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AlertManagerTest {

  private AlertManager alertManager;

  @BeforeEach
  void setUp() {
    HAL.initialize(500, 0);
    alertManager = AlertManager.getInstance();
    alertManager.clearDebounce();
  }

  @Test
  void testGetInstanceNotNull() {
    assertNotNull(AlertManager.getInstance());
  }

  @Test
  void testGetInstanceReturnsSameInstance() {
    assertSame(AlertManager.getInstance(), AlertManager.getInstance());
  }

  @Test
  void testActiveAlertsListNotNull() {
    assertNotNull(alertManager.getActiveAlerts());
  }

  @Test
  void testActiveAlertCountNonNegative() {
    assertTrue(alertManager.getActiveAlertCount() >= 0);
  }

  @Test
  void testCheckLoopTimeNormalValue() {
    assertDoesNotThrow(() -> alertManager.checkLoopTime(15.0));
  }

  @Test
  void testCheckLoopTimeWarningValue() {
    assertDoesNotThrow(() -> alertManager.checkLoopTime(25.0));
  }

  @Test
  void testCheckLoopTimeCriticalValue() {
    assertDoesNotThrow(() -> alertManager.checkLoopTime(100.0));
  }

  @Test
  void testCheckLoopTimeZero() {
    assertDoesNotThrow(() -> alertManager.checkLoopTime(0.0));
  }

  @Test
  void testCheckLoopTimeNegative() {
    assertDoesNotThrow(() -> alertManager.checkLoopTime(-5.0));
  }

  @Test
  void testClearDebounceDoesNotThrow() {
    assertDoesNotThrow(() -> alertManager.clearDebounce());
  }

  @Test
  void testClearDebounceIdempotent() {
    assertDoesNotThrow(
        () -> {
          alertManager.clearDebounce();
          alertManager.clearDebounce();
          alertManager.clearDebounce();
        });
  }

  @Test
  void testLogActiveAlertsDoesNotThrow() {
    assertDoesNotThrow(() -> alertManager.logActiveAlerts());
  }

  @Test
  void testBatteryCriticalBelowWarning() {
    assertTrue(
        AlertManager.BATTERY_CRITICAL_V < AlertManager.BATTERY_WARNING_V,
        "Critical voltage should be lower than warning voltage");
  }

  @Test
  void testMotorTempWarningBelowCritical() {
    assertTrue(
        AlertManager.MOTOR_TEMP_WARNING_C < AlertManager.MOTOR_TEMP_CRITICAL_C,
        "Warning temp should be lower than critical temp");
  }

  @Test
  void testLoopTimeWarningBelowError() {
    assertTrue(
        AlertManager.LOOP_TIME_WARNING_MS < AlertManager.LOOP_TIME_ERROR_MS,
        "Warning loop time should be lower than error loop time");
  }

  @Test
  void testCANUtilizationThresholdInRange() {
    assertTrue(
        AlertManager.CAN_UTILIZATION_WARNING > 0 && AlertManager.CAN_UTILIZATION_WARNING < 1.0,
        "CAN utilization should be a fraction between 0 and 1");
  }

  @Test
  void testBatteryThresholdsPositive() {
    assertTrue(AlertManager.BATTERY_CRITICAL_V > 0);
    assertTrue(AlertManager.BATTERY_WARNING_V > 0);
  }

  @Test
  void testBatteryHysteresisAboveWarning() {
    assertTrue(
        AlertManager.BATTERY_WARNING_CLEAR_V > AlertManager.BATTERY_WARNING_V,
        "Hysteresis clear voltage must be above warning voltage");
  }

  @Test
  void testMotorTempThresholdsPositive() {
    assertTrue(AlertManager.MOTOR_TEMP_WARNING_C > 0);
    assertTrue(AlertManager.MOTOR_TEMP_CRITICAL_C > 0);
  }

  @Test
  void testGetActiveAlertsReturnsCopy() {
    var list1 = alertManager.getActiveAlerts();
    var list2 = alertManager.getActiveAlerts();
    assertNotSame(list1, list2, "Should return a new list each call");
  }
}
