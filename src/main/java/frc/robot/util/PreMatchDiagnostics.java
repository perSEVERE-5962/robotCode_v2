package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.BatteryThresholds;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeActuator;
import frc.robot.subsystems.Shooter;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Pre-match diagnostics: SAFE mode (sensors only, while disabled) and FULL mode (actuator tests,
 * requires TEST mode).
 */
public class PreMatchDiagnostics {
  private static final PreMatchDiagnostics instance = new PreMatchDiagnostics();

  private static final String TRIGGER_SAFE_KEY = "Diagnostics/TriggerSafe";
  private static final String TRIGGER_FULL_KEY = "Diagnostics/TriggerFull";

  private static final double GYRO_SAMPLE_SECONDS = 2.0;
  private static final double VISION_SAMPLE_SECONDS = 3.0;
  private static final double DRIVE_TEST_SECONDS = 2.0;

  private static final double BATTERY_FAIL_V = BatteryThresholds.PRE_MATCH_FAIL_V;
  private static final double BATTERY_WARN_V = BatteryThresholds.PRE_MATCH_WARN_V;
  private static final double MOTOR_TEMP_WARN_C = 50.0;
  private static final double CAN_UTILIZATION_WARN = 0.50;
  private static final double GYRO_DRIFT_WARN_DEG = 1.0;
  private static final double GYRO_DRIFT_FAIL_DEG = 3.0;
  private static final double VISION_DROPOUT_WARN_PCT = 10.0;
  private static final double VISION_DROPOUT_FAIL_PCT = 30.0;
  private static final double SHOOTER_SPINUP_WARN_MS = 2000.0;
  private static final double SHOOTER_SPINUP_FAIL_MS = 3000.0;
  private static final double INDEXER_CURRENT_MIN_A = 2.0;
  private static final double DRIVE_RESPONSE_WARN_MS = 500.0;

  public enum CheckResult {
    PASS,
    WARN,
    FAIL,
    SKIPPED
  }

  public enum DiagnosticMode {
    SAFE,
    FULL
  }

  public record DiagnosticCheck(String name, CheckResult result, String message) {}

  private final List<DiagnosticCheck> lastResults = new ArrayList<>();
  private boolean allPassed = false;
  private int passCount = 0;
  private int warnCount = 0;
  private int failCount = 0;
  private int skippedCount = 0;
  private DiagnosticMode lastMode = DiagnosticMode.SAFE;

  private double startTimestamp = 0;
  private String currentStep = "IDLE";
  private double minBatteryVoltage = 15.0;
  private boolean brownoutOccurred = false;
  private int canBusOffStart = 0;
  private int canTxErrStart = 0;
  private int canRxErrStart = 0;

  private double gyroMaxDrift = 0;
  private double visionDropoutPct = 0;
  private double shooterSpinupMs = 0;
  private double indexerCurrentPeak = 0;
  private double driveResponseMs = 0;

  // Re-entry guard
  private boolean isRunning = false;

  private PreMatchDiagnostics() {
    SmartDashboard.putBoolean(TRIGGER_SAFE_KEY, false);
    SmartDashboard.putBoolean(TRIGGER_FULL_KEY, false);
  }

  public static PreMatchDiagnostics getInstance() {
    return instance;
  }

  /** Check if safe trigger was pressed and clear it. */
  public boolean checkAndClearSafeTrigger() {
    boolean triggered = SmartDashboard.getBoolean(TRIGGER_SAFE_KEY, false);
    if (triggered) {
      SmartDashboard.putBoolean(TRIGGER_SAFE_KEY, false);
    }
    return triggered;
  }

  /** Check if full trigger was pressed and clear it. */
  public boolean checkAndClearFullTrigger() {
    boolean triggered = SmartDashboard.getBoolean(TRIGGER_FULL_KEY, false);
    if (triggered) {
      SmartDashboard.putBoolean(TRIGGER_FULL_KEY, false);
    }
    return triggered;
  }

  public boolean isRunning() {
    return isRunning;
  }

  /**
   * Run SAFE diagnostic checks (sensor-only, no motor commands). Safe to run while disabled. Takes
   * ~5-6 seconds (gyro + vision sampling).
   */
  public void runSafeChecks() {
    if (isRunning) return;
    isRunning = true;
    lastMode = DiagnosticMode.SAFE;

    initializeRun();
    Logger.recordOutput("Diagnostics/Mode", "SAFE");

    // Safe checks - sensor only
    setStep("POWER");
    checkBattery();

    setStep("CAN");
    checkCANBus();
    checkMissingDevices();

    setStep("MOTORS");
    checkMotorTemps();
    checkMotorFaults();

    setStep("GYRO");
    checkGyroSampled(); // 2s sampling

    setStep("VISION");
    checkVisionSampled(); // 3s sampling

    setStep("DRIVE");
    checkDriveSensors(); // sensor-only, no motion

    // Skip actuator tests in safe mode
    addResult("ShooterSpinup", CheckResult.SKIPPED, "Requires TEST mode");
    addResult("IndexerCurrent", CheckResult.SKIPPED, "Requires TEST mode");
    addResult("DriveMotion", CheckResult.SKIPPED, "Requires TEST mode");

    finalizeRun("Safe Check");
  }

  /**
   * Run FULL diagnostic checks (includes actuator tests). REQUIRES TEST MODE - motors will move!
   * Takes ~12-15 seconds total.
   *
   * @return false if not in test mode (checks not run)
   */
  public boolean runFullChecks() {
    if (isRunning) return false;

    // Verify test mode
    if (!DriverStation.isTest() || !DriverStation.isEnabled()) {
      ElasticUtil.sendError(
          "Full Check Blocked", "Enable TEST mode first! Motors need to move.", 5000);
      Logger.recordOutput("Diagnostics/FullCheckBlocked", true);
      return false;
    }

    isRunning = true;
    lastMode = DiagnosticMode.FULL;

    initializeRun();
    Logger.recordOutput("Diagnostics/Mode", "FULL");
    Logger.recordOutput("Diagnostics/FullCheckBlocked", false);

    // All safe checks first
    setStep("POWER");
    checkBattery();

    setStep("CAN");
    checkCANBus();
    checkMissingDevices();

    setStep("MOTORS");
    checkMotorTemps();
    checkMotorFaults();

    setStep("GYRO");
    checkGyroSampled();

    setStep("VISION");
    checkVisionSampled();

    setStep("DRIVE");
    checkDriveSensors();

    // Actuator tests (test mode only)
    setStep("SHOOTER");
    checkShooterSpinup();

    setStep("INDEXER");
    checkIndexerCurrent();

    setStep("DRIVE_MOTION");
    checkDriveMotion();

    finalizeRun("Full Check");
    return true;
  }

  private void initializeRun() {
    lastResults.clear();
    passCount = 0;
    warnCount = 0;
    failCount = 0;
    skippedCount = 0;

    startTimestamp = Timer.getFPGATimestamp();
    minBatteryVoltage = RobotController.getBatteryVoltage();
    brownoutOccurred = false;
    gyroMaxDrift = 0;
    visionDropoutPct = 0;
    shooterSpinupMs = 0;
    indexerCurrentPeak = 0;
    driveResponseMs = 0;

    var canBaseline = RobotController.getCANStatus();
    canBusOffStart = canBaseline.busOffCount;
    canTxErrStart = canBaseline.transmitErrorCount;
    canRxErrStart = canBaseline.receiveErrorCount;

    Logger.recordOutput("Diagnostics/Running", true);
  }

  private void finalizeRun(String checkType) {
    setStep("COMPLETE");
    allPassed = (failCount == 0);
    logResults();
    sendNotification(checkType);
    Logger.recordOutput("Diagnostics/Running", false);
    isRunning = false;
  }

  private void setStep(String step) {
    currentStep = step;
    Logger.recordOutput("Diagnostics/Step", step);
    double v = RobotController.getBatteryVoltage();
    if (v < minBatteryVoltage) minBatteryVoltage = v;
    if (RobotController.isBrownedOut()) brownoutOccurred = true;
  }

  private void checkBattery() {
    double voltage = RobotController.getBatteryVoltage();

    if (voltage < BATTERY_FAIL_V) {
      addResult("Battery", CheckResult.FAIL, String.format("%.1fV - REPLACE BATTERY", voltage));
    } else if (voltage < BATTERY_WARN_V) {
      addResult(
          "Battery", CheckResult.WARN, String.format("%.1fV - Low, consider replacing", voltage));
    } else {
      addResult("Battery", CheckResult.PASS, String.format("%.1fV", voltage));
    }
  }

  private void checkCANBus() {
    var canStatus = RobotController.getCANStatus();

    int busOffDelta = canStatus.busOffCount - canBusOffStart;
    int txErrDelta = canStatus.transmitErrorCount - canTxErrStart;
    int rxErrDelta = canStatus.receiveErrorCount - canRxErrStart;

    Logger.recordOutput("Diagnostics/CANBusOffDelta", busOffDelta);
    Logger.recordOutput("Diagnostics/CANTxErrDelta", txErrDelta);
    Logger.recordOutput("Diagnostics/CANRxErrDelta", rxErrDelta);

    if (busOffDelta > 0) {
      addResult("CANBus", CheckResult.FAIL, "Bus off during check - check wiring");
    } else if (txErrDelta > 10 || rxErrDelta > 10) {
      addResult(
          "CANBus",
          CheckResult.WARN,
          String.format("CAN errors (TX:%d RX:%d)", txErrDelta, rxErrDelta));
    } else if (canStatus.percentBusUtilization > CAN_UTILIZATION_WARN) {
      addResult(
          "CANBus",
          CheckResult.WARN,
          String.format("%.0f%% utilization - high", canStatus.percentBusUtilization * 100));
    } else {
      addResult(
          "CANBus",
          CheckResult.PASS,
          String.format("%.0f%% utilization", canStatus.percentBusUtilization * 100));
    }
  }

  private void checkMissingDevices() {
    StringBuilder missing = new StringBuilder();

    try {
      if (Shooter.getInstance() == null) missing.append("Shooter,");
    } catch (Exception e) {
      missing.append("Shooter,");
    }

    try {
      if (Indexer.getInstance() == null) missing.append("Indexer,");
    } catch (Exception e) {
      missing.append("Indexer,");
    }

    try {
      if (Intake.getInstance() == null) missing.append("Intake,");
    } catch (Exception e) {
      missing.append("Intake,");
    }

    try {
      if (IntakeActuator.getInstance() == null) missing.append("IntakeActuator,");
    } catch (Exception e) {
      missing.append("IntakeActuator,");
    }

    try {
      if (Hanger.getInstance() == null) missing.append("Hanger,");
    } catch (Exception e) {
      missing.append("Hanger,");
    }

    String missingStr = missing.length() > 0 ? missing.substring(0, missing.length() - 1) : "none";

    Logger.recordOutput("Diagnostics/MissingDevices", missingStr);

    if (missing.length() > 0) {
      addResult("Devices", CheckResult.FAIL, "Missing: " + missingStr);
    } else {
      addResult("Devices", CheckResult.PASS, "All devices responding");
    }
  }

  private void checkMotorTemps() {
    try {
      double shooterTemp = Shooter.getInstance().getTemperature();
      double indexerTemp = Indexer.getInstance().getTemperature();
      double intakeTemp = Intake.getInstance().getTemperature();

      double maxTemp = Math.max(shooterTemp, Math.max(indexerTemp, intakeTemp));

      if (maxTemp > MOTOR_TEMP_WARN_C) {
        addResult(
            "MotorTemps", CheckResult.WARN, String.format("Max %.0f°C - allow cooling", maxTemp));
      } else {
        addResult("MotorTemps", CheckResult.PASS, String.format("Max %.0f°C", maxTemp));
      }
    } catch (Exception e) {
      addResult("MotorTemps", CheckResult.WARN, "Could not read temps");
    }
  }

  private void checkMotorFaults() {
    try {
      StringBuilder faults = new StringBuilder();

      int shooterFaults = Shooter.getInstance().getStickyFaultsRaw();
      int indexerFaults = Indexer.getInstance().getStickyFaultsRaw();
      int intakeFaults = Intake.getInstance().getStickyFaultsRaw();

      if (shooterFaults != 0)
        faults.append("Shooter(0x").append(Integer.toHexString(shooterFaults)).append("),");
      if (indexerFaults != 0)
        faults.append("Indexer(0x").append(Integer.toHexString(indexerFaults)).append("),");
      if (intakeFaults != 0)
        faults.append("Intake(0x").append(Integer.toHexString(intakeFaults)).append("),");

      Logger.recordOutput("Diagnostics/MotorFaults", faults.toString());

      if (faults.length() > 0) {
        addResult(
            "MotorFaults",
            CheckResult.WARN,
            "Sticky faults: " + faults.substring(0, faults.length() - 1));
      } else {
        addResult("MotorFaults", CheckResult.PASS, "No sticky faults");
      }
    } catch (Exception e) {
      addResult("MotorFaults", CheckResult.WARN, "Could not read faults");
    }
  }

  /** Sample gyro drift over 2 seconds while stationary. More reliable than single-instant read. */
  private void checkGyroSampled() {
    try {
      RobotContainer container = RobotContainer.getInstance();
      if (container == null || container.getSwerveSubsystem() == null) {
        addResult("Gyro", CheckResult.FAIL, "Drivebase not initialized");
        Logger.recordOutput("Diagnostics/GyroMaxDrift", 0.0);
        Logger.recordOutput("Diagnostics/GyroHealthy", false);
        return;
      }

      var swerve = container.getSwerveSubsystem();
      double startYaw = swerve.getHeading().getDegrees();
      double maxDrift = 0;
      double startTime = Timer.getFPGATimestamp();

      // Sample for GYRO_SAMPLE_SECONDS
      while ((Timer.getFPGATimestamp() - startTime) < GYRO_SAMPLE_SECONDS) {
        Timer.delay(0.05); // 50ms samples
        double currentYaw = swerve.getHeading().getDegrees();
        double drift = Math.abs(currentYaw - startYaw);
        if (drift > maxDrift) maxDrift = drift;

        // Track voltage
        double v = RobotController.getBatteryVoltage();
        if (v < minBatteryVoltage) minBatteryVoltage = v;
      }

      gyroMaxDrift = maxDrift;
      Logger.recordOutput("Diagnostics/GyroMaxDrift", gyroMaxDrift);

      if (maxDrift > GYRO_DRIFT_FAIL_DEG) {
        addResult(
            "Gyro",
            CheckResult.FAIL,
            String.format("Drift %.2f° in %.0fs - excessive", maxDrift, GYRO_SAMPLE_SECONDS));
        Logger.recordOutput("Diagnostics/GyroHealthy", false);
      } else if (maxDrift > GYRO_DRIFT_WARN_DEG) {
        addResult(
            "Gyro",
            CheckResult.WARN,
            String.format("Drift %.2f° in %.0fs - marginal", maxDrift, GYRO_SAMPLE_SECONDS));
        Logger.recordOutput("Diagnostics/GyroHealthy", false);
      } else {
        addResult(
            "Gyro",
            CheckResult.PASS,
            String.format("Drift %.2f° in %.0fs - stable", maxDrift, GYRO_SAMPLE_SECONDS));
        Logger.recordOutput("Diagnostics/GyroHealthy", true);
      }
    } catch (Exception e) {
      addResult("Gyro", CheckResult.WARN, "Error: " + e.getMessage());
      Logger.recordOutput("Diagnostics/GyroMaxDrift", 0.0);
      Logger.recordOutput("Diagnostics/GyroHealthy", false);
    }
  }

  /**
   * Sample vision frame drops over 3 seconds. Checks for consistent target detection when
   * stationary.
   */
  private void checkVisionSampled() {
    try {
      RobotContainer container = RobotContainer.getInstance();
      if (container == null || container.visionSubsystem == null) {
        addResult("Vision", CheckResult.WARN, "Vision not initialized");
        Logger.recordOutput("Diagnostics/VisionDropoutPct", 0.0);
        return;
      }

      var vision = container.visionSubsystem;
      int totalSamples = 0;
      int missedSamples = 0;
      double startTime = Timer.getFPGATimestamp();

      // Sample for VISION_SAMPLE_SECONDS
      while ((Timer.getFPGATimestamp() - startTime) < VISION_SAMPLE_SECONDS) {
        Timer.delay(0.05); // 50ms samples (20Hz)
        totalSamples++;

        // Check if any camera has a target
        boolean hasTarget = vision.hasTarget();
        if (!hasTarget) missedSamples++;
      }

      visionDropoutPct = (totalSamples > 0) ? (missedSamples * 100.0 / totalSamples) : 100.0;
      Logger.recordOutput("Diagnostics/VisionDropoutPct", visionDropoutPct);
      Logger.recordOutput("Diagnostics/VisionSamples", totalSamples);
      Logger.recordOutput("Diagnostics/VisionMissed", missedSamples);

      if (visionDropoutPct > VISION_DROPOUT_FAIL_PCT) {
        addResult(
            "Vision",
            CheckResult.FAIL,
            String.format("%.0f%% dropout - check cameras/lighting", visionDropoutPct));
      } else if (visionDropoutPct > VISION_DROPOUT_WARN_PCT) {
        addResult(
            "Vision",
            CheckResult.WARN,
            String.format("%.0f%% dropout - may affect aiming", visionDropoutPct));
      } else {
        addResult(
            "Vision",
            CheckResult.PASS,
            String.format("%.0f%% dropout - good tracking", visionDropoutPct));
      }
    } catch (Exception e) {
      addResult("Vision", CheckResult.WARN, "Error: " + e.getMessage());
      Logger.recordOutput("Diagnostics/VisionDropoutPct", 100.0);
    }
  }

  /** Check drive sensors without commanding motion. Verifies encoders and module CAN health. */
  private void checkDriveSensors() {
    try {
      RobotContainer container = RobotContainer.getInstance();
      if (container == null || container.getSwerveSubsystem() == null) {
        addResult("DriveSensors", CheckResult.FAIL, "Drivebase not initialized");
        return;
      }

      var swerve = container.getSwerveSubsystem();

      // Read module states to verify CAN communication
      var states = swerve.getSwerveDrive().getStates();
      boolean allResponding = states != null && states.length == 4;

      // Check for any obviously bad encoder values
      boolean encodersOK = true;
      if (states != null) {
        for (var state : states) {
          if (Double.isNaN(state.angle.getDegrees()) || Double.isNaN(state.speedMetersPerSecond)) {
            encodersOK = false;
            break;
          }
        }
      }

      Logger.recordOutput("Diagnostics/DriveModulesResponding", allResponding);
      Logger.recordOutput("Diagnostics/DriveEncodersOK", encodersOK);

      if (!allResponding) {
        addResult("DriveSensors", CheckResult.FAIL, "Not all modules responding");
      } else if (!encodersOK) {
        addResult("DriveSensors", CheckResult.WARN, "Encoder values suspect (NaN)");
      } else {
        addResult("DriveSensors", CheckResult.PASS, "All 4 modules responding");
      }
    } catch (Exception e) {
      addResult("DriveSensors", CheckResult.WARN, "Error: " + e.getMessage());
    }
  }

  private void checkShooterSpinup() {
    try {
      Shooter shooter = Shooter.getInstance();
      if (shooter == null) {
        shooterSpinupMs = -1;
        Logger.recordOutput("Diagnostics/ShooterSpinupMs", shooterSpinupMs);
        addResult("ShooterSpinup", CheckResult.FAIL, "Shooter not available");
        return;
      }

      double startTime = Timer.getFPGATimestamp();
      double targetRPM = shooter.getTunableTargetRPM();
      shooter.moveToVelocityWithPID(targetRPM);

      double timeout = SHOOTER_SPINUP_FAIL_MS / 1000.0 + 0.5;
      while (!shooter.isAtSpeed() && (Timer.getFPGATimestamp() - startTime) < timeout) {
        Timer.delay(0.02);
        double v = RobotController.getBatteryVoltage();
        if (v < minBatteryVoltage) minBatteryVoltage = v;
        if (RobotController.isBrownedOut()) brownoutOccurred = true;
      }

      shooterSpinupMs = (Timer.getFPGATimestamp() - startTime) * 1000.0;
      shooter.move(0);

      Logger.recordOutput("Diagnostics/ShooterSpinupMs", shooterSpinupMs);

      if (!shooter.isAtSpeed() || shooterSpinupMs > SHOOTER_SPINUP_FAIL_MS) {
        addResult(
            "ShooterSpinup",
            CheckResult.FAIL,
            String.format("%.0fms - too slow or failed", shooterSpinupMs));
      } else if (shooterSpinupMs > SHOOTER_SPINUP_WARN_MS) {
        addResult(
            "ShooterSpinup", CheckResult.WARN, String.format("%.0fms - slow", shooterSpinupMs));
      } else {
        addResult(
            "ShooterSpinup", CheckResult.PASS, String.format("%.0fms to speed", shooterSpinupMs));
      }
    } catch (Exception e) {
      shooterSpinupMs = -1;
      Logger.recordOutput("Diagnostics/ShooterSpinupMs", shooterSpinupMs);
      addResult("ShooterSpinup", CheckResult.FAIL, "Exception: " + e.getMessage());
    }
  }

  private void checkIndexerCurrent() {
    try {
      Indexer indexer = Indexer.getInstance();
      if (indexer == null) {
        indexerCurrentPeak = 0;
        Logger.recordOutput("Diagnostics/IndexerCurrentPeak", indexerCurrentPeak);
        addResult("IndexerCurrent", CheckResult.FAIL, "Indexer not available");
        return;
      }

      indexerCurrentPeak = 0;
      indexer.move(0.5);

      for (int i = 0; i < 50; i++) { // ~1 second
        Timer.delay(0.02);
        double current = indexer.getOutputCurrent();
        if (current > indexerCurrentPeak) indexerCurrentPeak = current;
      }

      indexer.move(0);

      Logger.recordOutput("Diagnostics/IndexerCurrentPeak", indexerCurrentPeak);

      if (indexerCurrentPeak < INDEXER_CURRENT_MIN_A) {
        addResult(
            "IndexerCurrent",
            CheckResult.WARN,
            String.format("%.1fA peak - motor may be disconnected", indexerCurrentPeak));
      } else {
        addResult(
            "IndexerCurrent",
            CheckResult.PASS,
            String.format("%.1fA peak current", indexerCurrentPeak));
      }
    } catch (Exception e) {
      indexerCurrentPeak = 0;
      Logger.recordOutput("Diagnostics/IndexerCurrentPeak", indexerCurrentPeak);
      addResult("IndexerCurrent", CheckResult.WARN, "Error: " + e.getMessage());
    }
  }

  /** Command small drive motion and verify response. Tests closed-loop control actually works. */
  private void checkDriveMotion() {
    try {
      RobotContainer container = RobotContainer.getInstance();
      if (container == null || container.getSwerveSubsystem() == null) {
        addResult("DriveMotion", CheckResult.FAIL, "Drivebase not initialized");
        Logger.recordOutput("Diagnostics/DriveResponseMs", -1.0);
        return;
      }

      var swerve = container.getSwerveSubsystem();

      var startPose = swerve.getPose();
      double startTime = Timer.getFPGATimestamp();

      // Command small forward motion (0.1 m/s for safety)
      swerve.drive(new edu.wpi.first.math.geometry.Translation2d(0.1, 0), 0, false);

      boolean motionDetected = false;
      double responseTime = 0;

      while ((Timer.getFPGATimestamp() - startTime) < DRIVE_TEST_SECONDS) {
        Timer.delay(0.02);
        var currentPose = swerve.getPose();
        double distanceMoved = currentPose.getTranslation().getDistance(startPose.getTranslation());

        if (distanceMoved > 0.01) { // 1cm threshold
          motionDetected = true;
          responseTime = (Timer.getFPGATimestamp() - startTime) * 1000.0;
          break;
        }
      }

      swerve.drive(new edu.wpi.first.math.geometry.Translation2d(0, 0), 0, false);

      driveResponseMs = responseTime;
      Logger.recordOutput("Diagnostics/DriveResponseMs", driveResponseMs);
      Logger.recordOutput("Diagnostics/DriveMotionDetected", motionDetected);

      if (!motionDetected) {
        addResult("DriveMotion", CheckResult.FAIL, "No motion detected - check modules");
      } else if (responseTime > DRIVE_RESPONSE_WARN_MS) {
        addResult(
            "DriveMotion",
            CheckResult.WARN,
            String.format("%.0fms response - sluggish", responseTime));
      } else {
        addResult("DriveMotion", CheckResult.PASS, String.format("%.0fms response", responseTime));
      }
    } catch (Exception e) {
      addResult("DriveMotion", CheckResult.WARN, "Error: " + e.getMessage());
      Logger.recordOutput("Diagnostics/DriveResponseMs", -1.0);
    }
  }

  private void addResult(String name, CheckResult result, String message) {
    lastResults.add(new DiagnosticCheck(name, result, message));

    switch (result) {
      case PASS -> passCount++;
      case WARN -> warnCount++;
      case FAIL -> failCount++;
      case SKIPPED -> skippedCount++;
    }
  }

  private void logResults() {
    double durationSec = Timer.getFPGATimestamp() - startTimestamp;

    Logger.recordOutput("Diagnostics/AllPassed", allPassed);
    Logger.recordOutput("Diagnostics/PassCount", passCount);
    Logger.recordOutput("Diagnostics/WarnCount", warnCount);
    Logger.recordOutput("Diagnostics/FailCount", failCount);
    Logger.recordOutput("Diagnostics/SkippedCount", skippedCount);
    Logger.recordOutput("Diagnostics/DurationSec", durationSec);
    Logger.recordOutput("Diagnostics/MinBatteryVoltage", minBatteryVoltage);
    Logger.recordOutput("Diagnostics/BrownoutOccurred", brownoutOccurred);

    for (DiagnosticCheck check : lastResults) {
      Logger.recordOutput(
          "Diagnostics/PreMatch/" + check.name + "/Passed", check.result == CheckResult.PASS);
      Logger.recordOutput("Diagnostics/PreMatch/" + check.name + "/Result", check.result.name());
      Logger.recordOutput("Diagnostics/PreMatch/" + check.name + "/Message", check.message);
    }

    StringBuilder summary = new StringBuilder();
    for (DiagnosticCheck check : lastResults) {
      String icon =
          switch (check.result) {
            case PASS -> "[OK]";
            case WARN -> "[WARN]";
            case FAIL -> "[FAIL]";
            case SKIPPED -> "[SKIP]";
          };
      summary
          .append(icon)
          .append(" ")
          .append(check.name)
          .append(": ")
          .append(check.message)
          .append("\n");
    }
    Logger.recordOutput("Diagnostics/Summary", summary.toString());
  }

  private void sendNotification(String checkType) {
    String modeNote = (lastMode == DiagnosticMode.SAFE) ? " (sensor-only)" : " (full)";

    if (failCount > 0) {
      ElasticUtil.sendError(
          checkType + " FAILED" + modeNote,
          String.format("%d fail, %d warn, %d skip", failCount, warnCount, skippedCount),
          10000);
    } else if (warnCount > 0) {
      ElasticUtil.sendWarning(
          checkType + modeNote,
          String.format("%d warnings, %d skipped", warnCount, skippedCount),
          5000);
    } else {
      ElasticUtil.sendInfo(
          checkType + " PASSED" + modeNote,
          skippedCount > 0
              ? String.format("All passed (%d skipped - enable TEST for full)", skippedCount)
              : "All systems ready",
          3000);
    }
  }

  public boolean isAllPassed() {
    return allPassed;
  }

  public List<DiagnosticCheck> getLastResults() {
    return new ArrayList<>(lastResults);
  }

  public int getFailCount() {
    return failCount;
  }

  public int getWarnCount() {
    return warnCount;
  }

  public int getSkippedCount() {
    return skippedCount;
  }

  public DiagnosticMode getLastMode() {
    return lastMode;
  }

  // Legacy compatibility - calls safe checks
  public void runAllChecks() {
    runSafeChecks();
  }

  public boolean checkAndClearTrigger() {
    return checkAndClearSafeTrigger();
  }
}
