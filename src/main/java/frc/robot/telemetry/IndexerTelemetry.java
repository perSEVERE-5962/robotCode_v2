package frc.robot.telemetry;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DeviceHealthConstants;
import frc.robot.Constants.StallDetectionConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.util.EventMarker;
import frc.robot.util.JamProtection;

/** Indexer telemetry: jam detection, stall tracking, PID audit. */
public class IndexerTelemetry implements SubsystemTelemetry {
  private Indexer indexer; // grabbed again in update() if not ready yet
  private boolean subsystemAvailable = false;

  private double jamConditionStartTime = 0;
  private boolean inJamCondition = false;
  private boolean jamDetected = false;
  private int totalJamCount = 0;

  private boolean running = false;
  private boolean wasRunning = false;
  private String direction = "STOPPED";
  private double appliedOutput = 0;
  private double currentAmps = 0;
  private double temperatureCelsius = 0;
  private double velocityRPM = 0;

  private boolean feederActive = false;
  private double jamFrequencyPerMin = 0;
  private double matchStartTime = -1;
  private boolean pidTuningEvent = false;
  private double prevKP = -1, prevKI = -1, prevKD = -1, prevFF = -1;

  private final Debouncer connectDebouncer =
      new Debouncer(DeviceHealthConstants.DISCONNECT_DEBOUNCE_SEC, Debouncer.DebounceType.kFalling);
  private boolean deviceConnected = false;
  private int deviceFaultsRaw = 0;

  private boolean stalled = false;
  private double stallStartTime = 0;
  private double stallDurationMs = 0;
  private boolean inStallCondition = false;

  private double targetSpeed = 0;
  private String activeCommandName = "none";

  private String jamProtectionState = "MONITORING";
  private int jamProtectionAttempts = 0;
  private boolean jamProtectionIntervening = false;

  private double prevAppliedOutput = 0;
  private int torqueReversalsThisSec = 0;
  private int torqueReversalCount = 0;
  private double torqueReversalWindowStart = 0;
  private boolean torqueReversalAlert = false;
  private static final int TORQUE_REVERSAL_ALERT_THRESHOLD = 5;

  public IndexerTelemetry() {
    this.indexer = Indexer.getInstance();
  }

  @Override
  public void update() {
    if (indexer == null) {
      indexer = Indexer.getInstance();
    }

    if (indexer == null) {
      subsystemAvailable = false;
      setDefaultValues();
      return;
    }

    subsystemAvailable = true;
    double now = Timer.getFPGATimestamp();

    try {
      appliedOutput = indexer.getAppliedOutput();
      wasRunning = running;
      running = indexer.isRunning();
      currentAmps = indexer.getOutputCurrent();
      temperatureCelsius = indexer.getTemperature();
      velocityRPM = indexer.getVelocity();

      targetSpeed = Indexer.getTunableTargetSpeed();
      deviceConnected = connectDebouncer.calculate(true);
      deviceFaultsRaw = indexer.getStickyFaultsRaw();
    } catch (Throwable t) {
      deviceConnected = connectDebouncer.calculate(false);
      deviceFaultsRaw = -1;
      setDefaultValues();
      return;
    }

    boolean meetsStallCriteria =
        (currentAmps > StallDetectionConstants.INDEXER_STALL_CURRENT_AMPS)
            && (Math.abs(velocityRPM) < StallDetectionConstants.INDEXER_STALL_VELOCITY_RPM)
            && running; // Only check when commanded to run

    if (meetsStallCriteria) {
      if (!inStallCondition) {
        stallStartTime = now;
        inStallCondition = true;
      }
      stallDurationMs = (now - stallStartTime) * 1000.0;
      stalled = (stallDurationMs >= StallDetectionConstants.INDEXER_STALL_DEBOUNCE_MS);
    } else {
      inStallCondition = false;
      stallDurationMs = 0;
      stalled = false;
    }

    if (appliedOutput > 0.05) {
      direction = "FORWARD";
    } else if (appliedOutput < -0.05) {
      direction = "REVERSE";
    } else {
      direction = "STOPPED";
    }

    double jamThreshold = Indexer.getJamCurrentThreshold();
    double jamTimeSec = Indexer.getJamTimeThreshold();
    boolean highCurrent = running && (currentAmps > jamThreshold);
    boolean wasJammed = jamDetected;

    if (highCurrent) {
      if (!inJamCondition) {
        jamConditionStartTime = now;
        inJamCondition = true;
      }
      jamDetected = (now - jamConditionStartTime) >= jamTimeSec;
    } else {
      inJamCondition = false;
      jamDetected = false;
    }

    if (jamDetected && !wasJammed) {
      totalJamCount++;
      SafeLog.run(() -> EventMarker.jamDetected("Indexer"));
    }

    feederActive = running && appliedOutput > 0.05 && !jamDetected;

    if (matchStartTime < 0) matchStartTime = now;
    double elapsed = now - matchStartTime;
    jamFrequencyPerMin = (elapsed > 0) ? (totalJamCount / elapsed) * 60.0 : 0;

    try {
      Command currentCmd = indexer.getCurrentCommand();
      activeCommandName = (currentCmd != null) ? currentCmd.getName() : "none";
    } catch (Throwable t) {
      activeCommandName = "unknown";
    }

    try {
      JamProtection jp = indexer.getJamProtection();
      jamProtectionState = jp.getState().name();
      jamProtectionAttempts = jp.getReverseAttempts();
      jamProtectionIntervening = jp.isIntervening();
    } catch (Throwable t) {
      jamProtectionState = "UNKNOWN";
    }

    boolean signChanged =
        (prevAppliedOutput > 0.01 && appliedOutput < -0.01)
            || (prevAppliedOutput < -0.01 && appliedOutput > 0.01);
    if (signChanged) {
      torqueReversalsThisSec++;
    }
    prevAppliedOutput = appliedOutput;

    if (now - torqueReversalWindowStart >= 1.0) {
      torqueReversalCount = torqueReversalsThisSec;
      torqueReversalsThisSec = 0;
      torqueReversalWindowStart = now;
    }
    torqueReversalAlert = torqueReversalCount >= TORQUE_REVERSAL_ALERT_THRESHOLD;

    pidTuningEvent = false;
    try {
      double curKP = Indexer.getTunableKP();
      double curKI = Indexer.getTunableKI();
      double curKD = Indexer.getTunableKD();
      double curFF = Indexer.getTunableFF();
      if (prevKP >= 0
          && (curKP != prevKP || curKI != prevKI || curKD != prevKD || curFF != prevFF)) {
        pidTuningEvent = true;
      }
      prevKP = curKP;
      prevKI = curKI;
      prevKD = curKD;
      prevFF = curFF;
    } catch (Throwable t) {
      pidTuningEvent = false;
    }
  }

  private void setDefaultValues() {
    running = false;
    direction = "STOPPED";
    appliedOutput = 0;
    currentAmps = 0;
    temperatureCelsius = 0;
    velocityRPM = 0;
    stalled = false;
    stallDurationMs = 0;
    inStallCondition = false;
    jamDetected = false;
    inJamCondition = false;
    torqueReversalAlert = false;
    targetSpeed = 0;
  }

  @Override
  public void log() {
    SafeLog.put("Indexer/Available", subsystemAvailable);
    SafeLog.put("Indexer/Running", running);
    SafeLog.put("Indexer/AppliedOutput", appliedOutput);
    SafeLog.put("Indexer/CurrentAmps", currentAmps);
    SafeLog.put("Indexer/TemperatureCelsius", temperatureCelsius);
    SafeLog.put("Indexer/VelocityRPM", velocityRPM);
    SafeLog.put("Indexer/Device/Connected", deviceConnected);
    SafeLog.put("Indexer/Stalled", stalled);
    SafeLog.put("Indexer/JamDetected", jamDetected);

    if (Constants.TUNING_MODE) {
      SafeLog.put("Indexer/PIDTuningEvent", pidTuningEvent);
      if (pidTuningEvent) {
        SafeLog.put("Indexer/Config/kP", prevKP);
        SafeLog.put("Indexer/Config/kI", prevKI);
        SafeLog.put("Indexer/Config/kD", prevKD);
        SafeLog.put("Indexer/Config/FF", prevFF);
      }
      SafeLog.put("Indexer/JamProtection/State", jamProtectionState);
      SafeLog.put("Indexer/JamProtection/Attempts", jamProtectionAttempts);
      SafeLog.put("Indexer/JamProtection/Intervening", jamProtectionIntervening);
      SafeLog.put("Indexer/TorqueReversalsPerSec", torqueReversalCount);
      SafeLog.put("Indexer/TorqueReversalAlert", torqueReversalAlert);
      SafeLog.put("Indexer/StallDurationMs", stallDurationMs);
      SafeLog.put("Indexer/Direction", direction);
      SafeLog.put("Indexer/FeederActive", feederActive);
      SafeLog.put("Indexer/ReadyToFire", !jamDetected);
      SafeLog.put("Indexer/TargetSpeed", targetSpeed);
      SafeLog.put("Indexer/TotalJamCount", totalJamCount);
      SafeLog.put("Indexer/JamFrequencyPerMin", jamFrequencyPerMin);
      SafeLog.put("Indexer/ActiveCommand", activeCommandName);
      SafeLog.put("Indexer/Device/FaultsRaw", deviceFaultsRaw);
    }
  }

  @Override
  public String getName() {
    return "Indexer";
  }

  public double getTemperature() {
    return temperatureCelsius;
  }

  public boolean isJamDetected() {
    return jamDetected;
  }

  public int getTotalJamCount() {
    return totalJamCount;
  }

  public boolean isStalled() {
    return stalled;
  }

  public boolean isJamProtectionIntervening() {
    return jamProtectionIntervening;
  }

  public boolean isDeviceConnected() {
    return deviceConnected;
  }

  public int getDeviceFaultsRaw() {
    return deviceFaultsRaw;
  }

  public String getActiveCommandName() {
    return activeCommandName;
  }

  public int getTorqueReversalsPerSec() {
    return torqueReversalCount;
  }

  public boolean isTorqueReversalAlert() {
    return torqueReversalAlert;
  }
}
