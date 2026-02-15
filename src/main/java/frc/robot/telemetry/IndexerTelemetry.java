package frc.robot.telemetry;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DeviceHealthConstants;
import frc.robot.Constants.StallDetectionConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.util.EventMarker;

/** Indexer telemetry: jam detection, stall tracking, PID audit. */
public class IndexerTelemetry implements SubsystemTelemetry {
  private Indexer indexer; // Not final - can re-acquire if null
  private boolean subsystemAvailable = false;

  // Jam detection (manual time-based so JamSeconds tunable is read each cycle)
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

  // Device health. Debounced to filter CAN bus transients
  private final Debouncer connectDebouncer =
      new Debouncer(DeviceHealthConstants.DISCONNECT_DEBOUNCE_SEC, Debouncer.DebounceType.kFalling);
  private boolean deviceConnected = false;
  private int deviceFaultsRaw = 0;

  private boolean stalled = false;
  private double stallStartTime = 0;
  private double stallDurationMs = 0;
  private boolean inStallCondition = false;

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
      velocityRPM = indexer.getVelocityRPM();

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

    // Direction
    if (appliedOutput > 0.05) {
      direction = "FORWARD";
    } else if (appliedOutput < -0.05) {
      direction = "REVERSE";
    } else {
      direction = "STOPPED";
    }

    // Cycle tracking: indexer started feeding (for CycleTracker wiring)
    if (running && !wasRunning && appliedOutput > 0.05) {
      SafeLog.run(() -> CycleTracker.getInstance().intakeComplete());
    }

    // Jam detection: manual time-based (reads both JamAmps and JamSeconds tunables live)
    double jamThreshold = indexer.getJamCurrentThreshold();
    double jamTimeSec = indexer.getJamTimeThreshold();
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

    // Count on rising edge only
    if (jamDetected && !wasJammed) {
      totalJamCount++;
      SafeLog.run(() -> EventMarker.jamDetected("Indexer"));
    }

    // Feeder active: running, outputting, not jammed
    feederActive = running && appliedOutput > 0.05 && !jamDetected;

    // Jam frequency over match duration
    if (matchStartTime < 0) matchStartTime = now;
    double elapsed = now - matchStartTime;
    jamFrequencyPerMin = (elapsed > 0) ? (totalJamCount / elapsed) * 60.0 : 0;

    // PID audit trail
    pidTuningEvent = false;
    try {
      double curKP = indexer.getTunableKP();
      double curKI = indexer.getTunableKI();
      double curKD = indexer.getTunableKD();
      double curFF = indexer.getTunableFF();
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
  }

  @Override
  public void log() {
    SafeLog.put("Indexer/Available", subsystemAvailable);
    SafeLog.put("Indexer/Running", running);
    SafeLog.put("Indexer/Direction", direction);
    SafeLog.put("Indexer/ReadyToFire", !jamDetected);
    SafeLog.put("Indexer/AppliedOutput", appliedOutput);
    SafeLog.put("Indexer/TargetSpeed", indexer != null ? indexer.getTunableTargetSpeed() : 0.0);
    SafeLog.put("Indexer/CurrentAmps", currentAmps);
    SafeLog.put("Indexer/TemperatureCelsius", temperatureCelsius);
    SafeLog.put("Indexer/JamDetected", jamDetected);
    SafeLog.put("Indexer/TotalJamCount", totalJamCount);

    SafeLog.put("Indexer/FeederActive", feederActive);
    SafeLog.put("Indexer/JamFrequencyPerMin", jamFrequencyPerMin);
    SafeLog.put("Indexer/PIDTuningEvent", pidTuningEvent);
    if (pidTuningEvent) {
      SafeLog.put("Indexer/Config/kP", prevKP);
      SafeLog.put("Indexer/Config/kI", prevKI);
      SafeLog.put("Indexer/Config/kD", prevKD);
      SafeLog.put("Indexer/Config/FF", prevFF);
    }

    // Device health
    SafeLog.put("Indexer/Device/Connected", deviceConnected);
    SafeLog.put("Indexer/Device/FaultsRaw", deviceFaultsRaw);

    SafeLog.put("Indexer/VelocityRPM", velocityRPM);
    SafeLog.put("Indexer/Stalled", stalled);
    SafeLog.put("Indexer/StallDurationMs", stallDurationMs);
  }

  @Override
  public String getName() {
    return "Indexer";
  }

  // Accessors for TelemetryManager
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

  public void clearJam() {
    jamDetected = false;
    inJamCondition = false;
  }

  public boolean isDeviceConnected() {
    return deviceConnected;
  }

  public int getDeviceFaultsRaw() {
    return deviceFaultsRaw;
  }
}
