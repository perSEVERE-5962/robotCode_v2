package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.BatteryThresholds;
import frc.robot.telemetry.TelemetryManager;
import org.littletonrobotics.junction.Logger;

/** Trend-based predictive alerts: voltage drop rate, temp rise rate, time-to-threshold. */
public class PredictiveAlerts {
  private static volatile PredictiveAlerts instance;

  // 50 samples = 1s at 50Hz on real hardware. In sim, loop runs slower (~22ms)
  // so this is ~1.1s of data,short enough that motor startup transients
  // cause false positives. Use 150 samples (~3s) in sim for better filtering.
  private static final int SAMPLE_SIZE = RobotBase.isSimulation() ? 150 : 50;

  private final double[] voltageSamples = new double[SAMPLE_SIZE];
  private int sampleIndex = 0;
  private boolean samplesInitialized = false;
  private double voltageDropRate = 0; // V/s
  private double predictedTimeToWarning = -1;
  private double predictedTimeToShutdown = -1;

  private final double[] tempSamples = new double[SAMPLE_SIZE];
  private double tempRate = 0; // C/s
  private double predictedTimeToOverheat = -1;

  private static final double VOLTAGE_WARNING = BatteryThresholds.WARNING_V;
  private static final double VOLTAGE_SHUTDOWN = BatteryThresholds.CRITICAL_V + 0.5;
  private static final double TEMP_OVERHEAT = 70.0;
  private static final double PREDICTION_WARN_SECONDS = 30.0;

  // Alert state (prevent spam)
  private boolean batteryAlertSent = false;
  private boolean tempAlertSent = false;

  private PredictiveAlerts() {
    double initialVoltage = RobotController.getBatteryVoltage();
    for (int i = 0; i < SAMPLE_SIZE; i++) {
      voltageSamples[i] = initialVoltage;
      tempSamples[i] = 25.0; // Assume room temp
    }
  }

  public static PredictiveAlerts getInstance() {
    if (instance == null) {
      instance = new PredictiveAlerts();
    }
    return instance;
  }

  /** Call once per loop cycle */
  public void update() {
    double voltage = RobotController.getBatteryVoltage();
    voltageSamples[sampleIndex] = voltage;

    TelemetryManager tm = TelemetryManager.getInstance();
    double maxTemp =
        Math.max(
            tm.getShooterTemperature(),
            Math.max(
                tm.getIndexerTemperature(),
                Math.max(tm.getIntakeTemperature(), tm.getIntakeActuatorTemperature())));
    tempSamples[sampleIndex] = maxTemp;

    sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;
    if (sampleIndex == 0) {
      samplesInitialized = true;
    }

    // Only calculate trends after we have a full window
    if (!samplesInitialized) {
      return;
    }

    voltageDropRate = -calculateSlope(voltageSamples); // negate so positive = dropping
    tempRate = calculateSlope(tempSamples);

    if (voltageDropRate > 0.001) {
      predictedTimeToWarning = (voltage - VOLTAGE_WARNING) / voltageDropRate;
      predictedTimeToShutdown = (voltage - VOLTAGE_SHUTDOWN) / voltageDropRate;
    } else {
      predictedTimeToWarning = -1;
      predictedTimeToShutdown = -1;
    }

    if (tempRate > 0.01) {
      predictedTimeToOverheat = (TEMP_OVERHEAT - maxTemp) / tempRate;
    } else {
      predictedTimeToOverheat = -1;
    }

    checkPredictiveAlerts();
  }

  /**
   * Calculate slope using simple linear regression. Returns change per second using actual loop
   * rate from TelemetryManager.
   */
  private double calculateSlope(double[] samples) {
    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    int n = SAMPLE_SIZE;

    for (int i = 0; i < n; i++) {
      sumX += i;
      sumY += samples[(sampleIndex + i) % SAMPLE_SIZE]; // Start from oldest
      sumXY += i * samples[(sampleIndex + i) % SAMPLE_SIZE];
      sumX2 += i * i;
    }

    double slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    // Convert to per-second using actual loop rate
    double loopTimeMs = TelemetryManager.getInstance().getLoopTimeMs();
    double samplesPerSecond = 1000.0 / Math.max(1.0, loopTimeMs);
    return slope * samplesPerSecond;
  }

  private void checkPredictiveAlerts() {
    // Battery prediction
    if (predictedTimeToWarning > 0 && predictedTimeToWarning < PREDICTION_WARN_SECONDS) {
      if (!batteryAlertSent) {
        ElasticUtil.sendWarning(
            "Battery Prediction",
            String.format(
                "Battery will hit warning in %.0fs at current drain", predictedTimeToWarning));
        batteryAlertSent = true;
      }
    } else {
      batteryAlertSent = false;
    }

    // Temp prediction
    if (predictedTimeToOverheat > 0 && predictedTimeToOverheat < PREDICTION_WARN_SECONDS) {
      if (!tempAlertSent) {
        ElasticUtil.sendWarning(
            "Temp Prediction",
            String.format(
                "Shooter will overheat in %.0fs at current rate", predictedTimeToOverheat));
        tempAlertSent = true;
      }
    } else {
      tempAlertSent = false;
    }
  }

  public void log() {
    Logger.recordOutput("Predict/VoltageDropRate", voltageDropRate);
    Logger.recordOutput("Predict/TimeToVoltageWarn", predictedTimeToWarning);
    Logger.recordOutput("Predict/TimeToShutdown", predictedTimeToShutdown);

    Logger.recordOutput("Predict/ShooterTempRate", tempRate);
    Logger.recordOutput("Predict/MaxMotorTempRate", tempRate);
    Logger.recordOutput("Predict/TimeToOverheat", predictedTimeToOverheat);

    Logger.recordOutput(
        "Predict/BatteryAtRisk",
        predictedTimeToWarning > 0 && predictedTimeToWarning < PREDICTION_WARN_SECONDS);
    Logger.recordOutput(
        "Predict/TempAtRisk",
        predictedTimeToOverheat > 0 && predictedTimeToOverheat < PREDICTION_WARN_SECONDS);
  }

  /** Reset for new match */
  public void reset() {
    samplesInitialized = false;
    sampleIndex = 0;
    batteryAlertSent = false;
    tempAlertSent = false;

    double initialVoltage = RobotController.getBatteryVoltage();
    for (int i = 0; i < SAMPLE_SIZE; i++) {
      voltageSamples[i] = initialVoltage;
      tempSamples[i] = 25.0;
    }
  }

  public boolean isBatteryAtRisk() {
    return predictedTimeToWarning > 0 && predictedTimeToWarning < PREDICTION_WARN_SECONDS;
  }

  public boolean isTempAtRisk() {
    return predictedTimeToOverheat > 0 && predictedTimeToOverheat < PREDICTION_WARN_SECONDS;
  }
}
