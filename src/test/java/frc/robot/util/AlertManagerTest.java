package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Shooter;
import frc.robot.telemetry.TelemetryManager;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests AlertManager's real alert logic: battery thresholds, hysteresis, disabled-only gating,
 * stall/jam propagation, and alert list accuracy.
 *
 * <p>Failure modes we protect against: - FALSE POSITIVE: team replaces good battery, investigates
 * non-existent jam - FALSE NEGATIVE: team goes to match with dead battery, misses stall -
 * HYSTERESIS BUG: alert oscillates at threshold boundary - STALE ALERT: condition clears but alert
 * stays active - DISABLED-ONLY BYPASS: battery warning fires mid-match (distracting)
 */
class AlertManagerTest {

  private static TelemetryManager tm;
  private AlertManager am;

  @BeforeAll
  static void initAll() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
    RoboRioSim.setVInVoltage(12.8);

    // Subsystems must exist for checkMotorTemps() / checkMotorStalls()
    Shooter.getInstance();
    Indexer.getInstance();
    IntakeRoller.getInstance();
    IntakePivot.getInstance();

    tm = TelemetryManager.getInstance();
  }

  @BeforeEach
  void setUp() throws Exception {
    am = AlertManager.getInstance();

    // Reset internal state (singleton persists across tests)
    setField(am, "inBatteryWarning", false);

    List<String> alerts = getField(am, "activeAlerts");
    alerts.clear();

    Map<String, Double> times = getField(am, "lastElasticNotifyTimes");
    times.clear();

    // Replace debouncer with fresh one (clears internal timing state)
    setField(am, "disabledDebouncer", new Debouncer(1.5, Debouncer.DebounceType.kRising));

    // Reset all WPILib Alert objects
    String[] alertFields = {
      "batteryLowAlert",
      "batteryCriticalAlert",
      "shooterTempAlert",
      "indexerTempAlert",
      "intakeRollerTempAlert",
      "intakePivotTempAlert",
      "canUtilizationAlert",
      "canBusOffAlert",
      "loopTimeWarningAlert",
      "loopTimeErrorAlert",
      "indexerJamAlert",
      "intakeJamAlert",
      "bandwidthWarningAlert",
      "bandwidthCriticalAlert",
      "brownoutRiskAlert",
      "shooterStallAlert",
      "indexerStallAlert",
      "intakeStallAlert"
    };
    for (String name : alertFields) {
      Alert alert = getField(am, name);
      alert.set(false);
    }

    // Clear telemetry stall/jam/brownout state from previous tests
    clearTelemetryState();

    // Healthy defaults
    RoboRioSim.setVInVoltage(12.8);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SafeLog.logAndReset();
  }

  @AfterAll
  static void tearDownAll() {
    closeSubsystemMotor("frc.robot.subsystems.Shooter");
    closeSubsystemMotor("frc.robot.subsystems.Indexer");
    closeSubsystemMotor("frc.robot.subsystems.IntakeRoller");
    closeSubsystemMotor("frc.robot.subsystems.IntakePivot");
    resetSingleton("frc.robot.subsystems.Shooter");
    resetSingleton("frc.robot.subsystems.Indexer");
    resetSingleton("frc.robot.subsystems.IntakeRoller");
    resetSingleton("frc.robot.subsystems.IntakePivot");
    resetSingleton("frc.robot.telemetry.TelemetryManager");
  }

  // Battery: false positives

  @Test
  void testNoBatteryAlertWhenVoltageHealthy() {
    satisfyDisabledDebouncer();
    RoboRioSim.setVInVoltage(12.8);
    am.checkAll();

    assertFalse(am.getActiveAlerts().contains("BatteryLow"), "No BatteryLow alert at 12.8V");
    assertFalse(
        am.getActiveAlerts().contains("BatteryCritical"), "No BatteryCritical alert at 12.8V");
    assertEquals(0, am.getActiveAlertCount(), "No alerts at healthy voltage in disabled mode");
  }

  @Test
  void testBatteryWarningSuppressedWhenEnabled() {
    // Mid-match voltage sag (11.0V) should NOT trigger warning
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    RoboRioSim.setVInVoltage(11.0);
    am.checkAll();

    assertFalse(
        am.getActiveAlerts().contains("BatteryLow"),
        "Battery warning must not fire mid-match (voltage sag is normal)");
  }

  // Battery: false negatives

  @Test
  void testBatteryWarningFiresWhenLowAndDisabled() {
    satisfyDisabledDebouncer();
    RoboRioSim.setVInVoltage(11.0);
    am.checkAll();

    assertTrue(
        am.getActiveAlerts().contains("BatteryLow"),
        "Must warn about 11.0V battery when disabled (pre-match)");
  }

  @Test
  void testBatteryCriticalFiresRegardlessOfMode() {
    // Critical fires even when ENABLED (brownout danger is immediate)
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    RoboRioSim.setVInVoltage(9.5);
    am.checkAll();

    assertTrue(
        am.getActiveAlerts().contains("BatteryCritical"),
        "CRITICAL must fire regardless of enabled/disabled state");
  }

  @Test
  void testCriticalOverridesWarning() throws Exception {
    satisfyDisabledDebouncer();
    RoboRioSim.setVInVoltage(9.0);
    am.checkAll();

    assertTrue(am.getActiveAlerts().contains("BatteryCritical"));
    assertFalse(
        am.getActiveAlerts().contains("BatteryLow"),
        "Critical and Warning should not both be active (confusing)");

    Alert critAlert = getField(am, "batteryCriticalAlert");
    Alert warnAlert = getField(am, "batteryLowAlert");
    assertTrue(critAlert.get());
    assertFalse(warnAlert.get());
  }

  // Battery: hysteresis

  @Test
  void testHysteresisPreventsPrematureClearing() throws Exception {
    // Battery dips to 11.0V, then "recovers" to 11.7V (still bad)
    satisfyDisabledDebouncer();
    RoboRioSim.setVInVoltage(11.0);
    am.checkAll();
    assertTrue(am.getActiveAlerts().contains("BatteryLow"), "Warning should fire at 11.0V");

    // Partial recovery: above WARNING_V (11.5) but below CLEAR_V (12.0)
    RoboRioSim.setVInVoltage(11.7);
    am.checkAll();
    assertTrue(
        am.getActiveAlerts().contains("BatteryLow"),
        "Warning must persist at 11.7V (below hysteresis clear threshold 12.0V)");

    boolean inWarning = getField(am, "inBatteryWarning");
    assertTrue(inWarning, "Hysteresis flag should still be set");
  }

  @Test
  void testHysteresisClearsAboveThreshold() throws Exception {
    // Establish warning state
    satisfyDisabledDebouncer();
    RoboRioSim.setVInVoltage(11.0);
    am.checkAll();
    assertTrue(am.getActiveAlerts().contains("BatteryLow"));

    // Full recovery above CLEAR_V (12.0)
    RoboRioSim.setVInVoltage(12.5);
    am.checkAll();
    assertFalse(
        am.getActiveAlerts().contains("BatteryLow"),
        "Warning must clear after voltage recovers above 12.0V");

    boolean inWarning = getField(am, "inBatteryWarning");
    assertFalse(inWarning, "Hysteresis flag should clear");
  }

  @Test
  void testHysteresisExactBoundaryClears() throws Exception {
    // Establish warning state
    satisfyDisabledDebouncer();
    RoboRioSim.setVInVoltage(11.0);
    am.checkAll();

    // Exactly at CLEAR_V (12.0): code checks voltage < 12.0, so 12.0 is NOT < 12.0
    RoboRioSim.setVInVoltage(12.0);
    am.checkAll();
    assertFalse(
        am.getActiveAlerts().contains("BatteryLow"),
        "Exactly 12.0V should clear warning (12.0 is not < 12.0)");
  }

  // Battery: debouncer gate

  @Test
  void testBatteryWarningRequiresDebouncedDisabled() {
    // Disabled for < 1.5s (brief mode transition) should NOT trigger
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    // Do NOT step time past debounce period
    RoboRioSim.setVInVoltage(11.0);
    am.checkAll();

    assertFalse(
        am.getActiveAlerts().contains("BatteryLow"),
        "Warning must not fire before 1.5s disabled debounce (prevents false alarm on mode transition)");
  }

  @Test
  void testBatteryWarningFiresAfterDebouncePeriod() {
    satisfyDisabledDebouncer();
    RoboRioSim.setVInVoltage(11.0);
    am.checkAll();

    assertTrue(
        am.getActiveAlerts().contains("BatteryLow"),
        "Warning must fire after 1.5s debounce satisfied");
  }

  // Alert list accuracy

  @Test
  void testCheckAllClearsAlertListEachCall() {
    // First call: low battery triggers alert
    satisfyDisabledDebouncer();
    RoboRioSim.setVInVoltage(11.0);
    am.checkAll();
    assertTrue(am.getActiveAlertCount() > 0, "Should have alerts at low voltage");

    // Second call: voltage recovered
    RoboRioSim.setVInVoltage(12.8);
    am.checkAll();
    assertFalse(
        am.getActiveAlerts().contains("BatteryLow"),
        "Recovered alerts must not persist from previous cycle");
  }

  @Test
  void testGetActiveAlertsReturnsCopy() {
    am.checkAll();
    List<String> list1 = am.getActiveAlerts();
    List<String> list2 = am.getActiveAlerts();
    assertNotSame(list1, list2, "Must return defensive copy");
  }

  // Stall alerts via telemetry

  @Test
  void testShooterStallAlertActivates() throws Exception {
    // Inject stall via TelemetryManager -> shooterTelemetry
    Object shooterTel = getTelemetryField("shooterTelemetry");
    setField(shooterTel, "stalled", true);

    am.checkAll();

    assertTrue(
        am.getActiveAlerts().contains("ShooterStall"),
        "ShooterStall alert must fire when telemetry reports stall");
    Alert stallAlert = getField(am, "shooterStallAlert");
    assertTrue(stallAlert.get());
  }

  @Test
  void testShooterStallAlertClearsOnRecovery() throws Exception {
    // Activate
    Object shooterTel = getTelemetryField("shooterTelemetry");
    setField(shooterTel, "stalled", true);
    am.checkAll();
    assertTrue(am.getActiveAlerts().contains("ShooterStall"));

    // Recover
    setField(shooterTel, "stalled", false);
    am.checkAll();
    assertFalse(
        am.getActiveAlerts().contains("ShooterStall"),
        "ShooterStall must clear when stall condition resolves");
    Alert stallAlert = getField(am, "shooterStallAlert");
    assertFalse(stallAlert.get());
  }

  @Test
  void testIndexerStallAlertActivates() throws Exception {
    Object indexerTel = getTelemetryField("indexerTelemetry");
    setField(indexerTel, "stalled", true);
    am.checkAll();

    assertTrue(am.getActiveAlerts().contains("IndexerStall"));
  }

  @Test
  void testIntakeStallAlertActivates() throws Exception {
    Object intakeRollerTel = getTelemetryField("intakeRollerTelemetry");
    setField(intakeRollerTel, "stalled", true);
    am.checkAll();

    assertTrue(am.getActiveAlerts().contains("IntakeStall"));
  }

  // Jam alerts via telemetry

  @Test
  void testIndexerJamAlertActivates() throws Exception {
    Object indexerTel = getTelemetryField("indexerTelemetry");
    setField(indexerTel, "jamDetected", true);
    am.checkAll();

    assertTrue(
        am.getActiveAlerts().contains("IndexerJam"),
        "IndexerJam alert must fire when telemetry reports jam");
    Alert jamAlert = getField(am, "indexerJamAlert");
    assertTrue(jamAlert.get());
  }

  @Test
  void testIndexerJamAlertClears() throws Exception {
    Object indexerTel = getTelemetryField("indexerTelemetry");
    setField(indexerTel, "jamDetected", true);
    am.checkAll();
    assertTrue(am.getActiveAlerts().contains("IndexerJam"));

    setField(indexerTel, "jamDetected", false);
    am.checkAll();
    assertFalse(
        am.getActiveAlerts().contains("IndexerJam"), "IndexerJam must clear when jam resolves");
  }

  @Test
  void testIntakeJamAlertActivates() throws Exception {
    Object intakeRollerTel = getTelemetryField("intakeRollerTelemetry");
    setField(intakeRollerTel, "jamDetected", true);
    am.checkAll();

    assertTrue(am.getActiveAlerts().contains("IntakeJam"));
  }

  // Brownout risk

  @Test
  void testBrownoutRiskAlertActivates() throws Exception {
    Object sysTel = getTelemetryField("systemHealthTelemetry");
    setField(sysTel, "brownoutRisk", true);
    am.checkAll();

    assertTrue(
        am.getActiveAlerts().contains("BrownoutRisk"),
        "BrownoutRisk must fire when SystemHealthTelemetry reports risk");
  }

  @Test
  void testBrownoutRiskAlertClears() throws Exception {
    Object sysTel = getTelemetryField("systemHealthTelemetry");
    setField(sysTel, "brownoutRisk", true);
    am.checkAll();
    assertTrue(am.getActiveAlerts().contains("BrownoutRisk"));

    setField(sysTel, "brownoutRisk", false);
    am.checkAll();
    assertFalse(am.getActiveAlerts().contains("BrownoutRisk"));
  }

  // Elastic debounce

  @Test
  void testClearDebounceResetsTimers() throws Exception {
    // Trigger a notification to populate the map
    satisfyDisabledDebouncer();
    RoboRioSim.setVInVoltage(11.0);
    am.checkAll();

    Map<String, Double> times = getField(am, "lastElasticNotifyTimes");
    assertFalse(times.isEmpty(), "Should have notification timestamps after alert");

    am.clearDebounce();
    assertTrue(times.isEmpty(), "clearDebounce() must reset all notification timers");
  }

  // No false alarms on healthy state

  @Test
  void testNoAlertsOnHealthyState() throws Exception {
    // Ensure TelemetryManager sub-objects report healthy state
    clearTelemetryState();

    RoboRioSim.setVInVoltage(12.8);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    am.checkAll();

    List<String> alerts = am.getActiveAlerts();
    assertEquals(0, alerts.size(), "Healthy robot should have zero active alerts, got: " + alerts);
  }

  // Helpers

  private void satisfyDisabledDebouncer() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(2.0);
    // Debouncer needs a calculate() call after time advance
    // checkAll() -> checkBattery() -> disabledDebouncer.calculate() handles this
  }

  private void clearTelemetryState() throws Exception {
    String[] telFields = {"shooterTelemetry", "indexerTelemetry", "intakeRollerTelemetry"};
    for (String tf : telFields) {
      Object tel = getTelemetryField(tf);
      if (tel != null) {
        try {
          setField(tel, "stalled", false);
        } catch (Exception e) {
        }
        try {
          setField(tel, "jamDetected", false);
        } catch (Exception e) {
        }
      }
    }
    Object sysTel = getTelemetryField("systemHealthTelemetry");
    if (sysTel != null) {
      try {
        setField(sysTel, "brownoutRisk", false);
      } catch (Exception e) {
      }
    }
  }

  private Object getTelemetryField(String fieldName) throws Exception {
    Field f = TelemetryManager.class.getDeclaredField(fieldName);
    f.setAccessible(true);
    return f.get(tm);
  }

  @SuppressWarnings("unchecked")
  private <T> T getField(Object obj, String fieldName) throws Exception {
    Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    return (T) f.get(obj);
  }

  private void setField(Object obj, String fieldName, Object value) throws Exception {
    Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    f.set(obj, value);
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

  private static void closeSubsystemMotor(String className) {
    try {
      Class<?> clazz = Class.forName(className);
      Field f = clazz.getDeclaredField("instance");
      f.setAccessible(true);
      Object instance = f.get(null);
      if (instance != null) {
        Method getMotor = clazz.getMethod("getMotor");
        Object motor = getMotor.invoke(instance);
        if (motor instanceof AutoCloseable) {
          ((AutoCloseable) motor).close();
        }
        try {
          Field followersField = clazz.getDeclaredField("followers");
          followersField.setAccessible(true);
          Object[] followers = (Object[]) followersField.get(instance);
          if (followers != null) {
            for (Object follower : followers) {
              if (follower instanceof AutoCloseable) {
                ((AutoCloseable) follower).close();
              }
            }
          }
        } catch (NoSuchFieldException e2) {
        }
      }
    } catch (Exception e) {
    }
  }
}
