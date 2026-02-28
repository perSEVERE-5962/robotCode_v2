package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.BatteryThresholds;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeActuator;
import frc.robot.subsystems.Shooter;
import frc.robot.telemetry.TelemetryManager;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized alert management for robot health monitoring. Monitors battery, motor temps, CAN bus,
 * and loop timing. Elastic notifications are debounced to prevent spam.
 */
public class AlertManager {
  private static final AlertManager instance = new AlertManager();

  // Alert thresholds
  public static final double BATTERY_CRITICAL_V = BatteryThresholds.CRITICAL_V;
  public static final double BATTERY_WARNING_V = BatteryThresholds.WARNING_V;
  public static final double MOTOR_TEMP_CRITICAL_C = 80.0;
  public static final double MOTOR_TEMP_WARNING_C = 65.0;
  public static final double CAN_UTILIZATION_WARNING = 0.70;
  public static final double LOOP_TIME_WARNING_MS = 20.0;
  public static final double LOOP_TIME_ERROR_MS = 40.0;

  // Debounce timing (seconds)
  private static final double DEBOUNCE_TIME_S = 5.0;
  private static final double BATTERY_DEBOUNCE_TIME_S = 30.0;

  // Hysteresis: once battery warning triggers, don't clear until voltage rises above this
  public static final double BATTERY_WARNING_CLEAR_V = 12.0;
  private boolean inBatteryWarning = false;

  // WPILib Alerts for Driver Station
  private final Alert batteryLowAlert = new Alert("Battery voltage low", AlertType.kWarning);
  private final Alert batteryCriticalAlert =
      new Alert("BATTERY CRITICAL - Replace now!", AlertType.kError);
  private final Alert shooterTempAlert = new Alert("Shooter motor overheating", AlertType.kWarning);
  private final Alert indexerTempAlert = new Alert("Indexer motor overheating", AlertType.kWarning);
  private final Alert intakeTempAlert = new Alert("Intake motor overheating", AlertType.kWarning);
  private final Alert intakeActuatorTempAlert =
      new Alert("IntakeActuator motor overheating", AlertType.kWarning);
  private final Alert canUtilizationAlert =
      new Alert("CAN bus utilization high", AlertType.kWarning);
  private final Alert canBusOffAlert = new Alert("CAN bus error detected", AlertType.kError);
  private final Alert loopTimeWarningAlert =
      new Alert("Loop time exceeding 20ms", AlertType.kWarning);
  private final Alert loopTimeErrorAlert = new Alert("Loop time critical >40ms", AlertType.kError);
  private final Alert indexerJamAlert = new Alert("Indexer jam detected", AlertType.kWarning);
  private final Alert intakeJamAlert = new Alert("Intake jam detected", AlertType.kWarning);
  private final Alert bandwidthWarningAlert =
      new Alert("Network bandwidth > 70%", AlertType.kWarning);
  private final Alert bandwidthCriticalAlert =
      new Alert("Network bandwidth > 90% - REDUCE CAMERA!", AlertType.kError);

  private final Alert brownoutRiskAlert =
      new Alert("BROWNOUT IMMINENT - reduce power!", AlertType.kError);
  private final Alert shooterStallAlert = new Alert("Shooter motor stalled", AlertType.kWarning);
  private final Alert indexerStallAlert = new Alert("Indexer motor stalled", AlertType.kWarning);
  private final Alert intakeStallAlert = new Alert("Intake motor stalled", AlertType.kWarning);

  private final List<String> activeAlerts = new ArrayList<>();

  // Elastic notification debouncing
  private final Map<String, Double> lastElasticNotifyTimes = new HashMap<>();

  private AlertManager() {}

  public static AlertManager getInstance() {
    return instance;
  }

  /**
   * Runs all alert checks except loop time. Loop time checked separately via checkLoopTime(). Call
   * logActiveAlerts() after both.
   */
  public void checkAll() {
    activeAlerts.clear();

    checkBattery();
    checkBrownoutRisk();
    checkMotorTemps();
    checkMotorStalls();
    checkCANBus();
    checkJams();
    checkBandwidth();
  }

  /** Log active alerts. Call after checkAll() and checkLoopTime() complete. */
  public void logActiveAlerts() {
    Logger.recordOutput("Alerts/ActiveCount", activeAlerts.size());
    Logger.recordOutput(
        "Alerts/ActiveList", activeAlerts.isEmpty() ? "none" : String.join(", ", activeAlerts));
  }

  private void checkBattery() {
    double voltage = RobotController.getBatteryVoltage();

    if (voltage < BATTERY_CRITICAL_V) {
      inBatteryWarning = true;
      batteryCriticalAlert.set(true);
      batteryLowAlert.set(false);
      addAlert("BatteryCritical");
      notifyElastic(
          "BatteryCritical",
          "Battery Critical",
          String.format("Battery at %.1fV - REPLACE NOW!", voltage),
          true);
    } else if (voltage < BATTERY_WARNING_V
        || (inBatteryWarning && voltage < BATTERY_WARNING_CLEAR_V)) {
      inBatteryWarning = true;
      batteryCriticalAlert.set(false);
      batteryLowAlert.set(true);
      addAlert("BatteryLow");
      notifyElastic("BatteryLow", "Battery Low", String.format("Battery at %.1fV", voltage), false);
    } else {
      inBatteryWarning = false;
      batteryCriticalAlert.set(false);
      batteryLowAlert.set(false);
    }
  }

  private void checkBrownoutRisk() {
    try {
      TelemetryManager tm = TelemetryManager.getInstance();
      if (tm.isBrownoutRisk()) {
        brownoutRiskAlert.set(true);
        addAlert("BrownoutRisk");
        notifyElastic(
            "BrownoutRisk",
            "BROWNOUT IMMINENT",
            String.format(
                "Voltage dropping fast at %.1fV - reduce power!",
                RobotController.getBatteryVoltage()),
            true);
      } else {
        brownoutRiskAlert.set(false);
      }
    } catch (Throwable t) {
      brownoutRiskAlert.set(false);
    }
  }

  private void checkMotorTemps() {
    try {
      Shooter shooter = Shooter.getInstance();
      if (shooter != null) {
        double shooterTemp = shooter.getTemperature();
        if (shooterTemp > MOTOR_TEMP_WARNING_C) {
          shooterTempAlert.set(true);
          addAlert("ShooterOverheat");
          if (shooterTemp > MOTOR_TEMP_CRITICAL_C) {
            notifyElastic(
                "ShooterOverheat",
                "Shooter Overheating",
                String.format("Shooter at %.0f°C - CRITICAL!", shooterTemp),
                true);
          }
        } else {
          shooterTempAlert.set(false);
        }
      }
    } catch (Throwable t) {
      shooterTempAlert.set(false);
    }

    try {
      Indexer indexer = Indexer.getInstance();
      if (indexer != null) {
        double indexerTemp = indexer.getTemperature();
        if (indexerTemp > MOTOR_TEMP_WARNING_C) {
          indexerTempAlert.set(true);
          addAlert("IndexerOverheat");
        } else {
          indexerTempAlert.set(false);
        }
      }
    } catch (Throwable t) {
      indexerTempAlert.set(false);
    }

    try {
      Intake intake = Intake.getInstance();
      if (intake != null) {
        double intakeTemp = intake.getTemperature();
        if (intakeTemp > MOTOR_TEMP_WARNING_C) {
          intakeTempAlert.set(true);
          addAlert("IntakeOverheat");
        } else {
          intakeTempAlert.set(false);
        }
      }
    } catch (Throwable t) {
      intakeTempAlert.set(false);
    }

    try {
      IntakeActuator intakeActuator = IntakeActuator.getInstance();
      if (intakeActuator != null) {
        double intakeActuatorTemp = intakeActuator.getTemperature();
        if (intakeActuatorTemp > MOTOR_TEMP_WARNING_C) {
          intakeActuatorTempAlert.set(true);
          addAlert("IntakeActuatorOverheat");
        } else {
          intakeActuatorTempAlert.set(false);
        }
      }
    } catch (Throwable t) {
      intakeActuatorTempAlert.set(false);
    }
  }

  private void checkMotorStalls() {
    try {
      TelemetryManager tm = TelemetryManager.getInstance();

      if (tm.isShooterStalled()) {
        shooterStallAlert.set(true);
        addAlert("ShooterStall");
        notifyElastic(
            "ShooterStall",
            "Shooter Stalled",
            "Shooter motor stalled - check for obstruction",
            true);
      } else {
        shooterStallAlert.set(false);
      }

      if (tm.isIndexerStalled()) {
        indexerStallAlert.set(true);
        addAlert("IndexerStall");
        notifyElastic(
            "IndexerStall",
            "Indexer Stalled",
            "Indexer motor stalled - check for obstruction",
            true);
      } else {
        indexerStallAlert.set(false);
      }

      if (tm.isIntakeStalled()) {
        intakeStallAlert.set(true);
        addAlert("IntakeStall");
        notifyElastic(
            "IntakeStall", "Intake Stalled", "Intake motor stalled - check for obstruction", true);
      } else {
        intakeStallAlert.set(false);
      }
    } catch (Throwable t) {
      shooterStallAlert.set(false);
      indexerStallAlert.set(false);
      intakeStallAlert.set(false);
    }
  }

  private void checkCANBus() {
    var canStatus = RobotController.getCANStatus();

    if (canStatus.busOffCount > 0) {
      canBusOffAlert.set(true);
      canUtilizationAlert.set(false);
      addAlert("CANBusOff");
      notifyElastic("CANBusOff", "CAN Bus Error", "CAN bus off detected - check wiring!", true);
    } else if (canStatus.percentBusUtilization > CAN_UTILIZATION_WARNING) {
      canBusOffAlert.set(false);
      canUtilizationAlert.set(true);
      addAlert("CANUtilizationHigh");
    } else {
      canBusOffAlert.set(false);
      canUtilizationAlert.set(false);
    }
  }

  /** Check loop time alerts. Called from Robot.java with actual timing data. */
  public void checkLoopTime(double loopTimeMs) {
    if (loopTimeMs > LOOP_TIME_ERROR_MS) {
      loopTimeErrorAlert.set(true);
      loopTimeWarningAlert.set(false);
      addAlert("LoopTimeCritical");
    } else if (loopTimeMs > LOOP_TIME_WARNING_MS) {
      loopTimeErrorAlert.set(false);
      loopTimeWarningAlert.set(true);
      addAlert("LoopTimeWarning");
    } else {
      loopTimeErrorAlert.set(false);
      loopTimeWarningAlert.set(false);
    }
  }

  private void checkJams() {
    if (TelemetryManager.getInstance().isIndexerJamDetected()) {
      indexerJamAlert.set(true);
      addAlert("IndexerJam");
      notifyElastic("IndexerJam", "Indexer Jam", "Indexer jam detected - clear manually", true);
    } else {
      indexerJamAlert.set(false);
    }

    if (TelemetryManager.getInstance().isIntakeJamDetected()) {
      intakeJamAlert.set(true);
      addAlert("IntakeJam");
      notifyElastic("IntakeJam", "Intake Jam", "Intake jam detected - clear manually", true);
    } else {
      intakeJamAlert.set(false);
    }
  }

  private void checkBandwidth() {
    TelemetryManager tm = TelemetryManager.getInstance();

    if (tm.isBandwidthCritical()) {
      bandwidthCriticalAlert.set(true);
      bandwidthWarningAlert.set(false);
      addAlert("BandwidthCritical");
      notifyElastic(
          "BandwidthCritical",
          "Bandwidth Critical",
          String.format("%.0f%% - REDUCE CAMERA RESOLUTION!", tm.getBandwidthPercent()),
          true);
    } else if (tm.isBandwidthWarning()) {
      bandwidthCriticalAlert.set(false);
      bandwidthWarningAlert.set(true);
      addAlert("BandwidthWarning");
    } else {
      bandwidthCriticalAlert.set(false);
      bandwidthWarningAlert.set(false);
    }
  }

  private void addAlert(String alertName) {
    activeAlerts.add(alertName);
  }

  /** Sends notification to Elastic with debouncing. Battery alerts use longer cooldown. */
  private void notifyElastic(String key, String title, String message, boolean isError) {
    double now = Timer.getFPGATimestamp();
    Double lastTime = lastElasticNotifyTimes.get(key);
    boolean isBatteryKey = key.startsWith("Battery");
    double debounce = isBatteryKey ? BATTERY_DEBOUNCE_TIME_S : DEBOUNCE_TIME_S;

    if (lastTime == null || (now - lastTime) > debounce) {
      if (isError) {
        ElasticUtil.sendError(title, message);
      } else {
        ElasticUtil.sendWarning(title, message);
      }
      lastElasticNotifyTimes.put(key, now);
    }
  }

  /** Clears all debounce timers. Call on mode transitions if desired. */
  public void clearDebounce() {
    lastElasticNotifyTimes.clear();
  }

  public List<String> getActiveAlerts() {
    return new ArrayList<>(activeAlerts);
  }

  public int getActiveAlertCount() {
    return activeAlerts.size();
  }
}
