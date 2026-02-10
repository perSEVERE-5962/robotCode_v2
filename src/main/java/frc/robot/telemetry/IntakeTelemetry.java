package frc.robot.telemetry;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DeviceHealthConstants;
import frc.robot.Constants.StallDetectionConstants;
import frc.robot.subsystems.Intake;
import frc.robot.util.EventMarker;

/** Intake telemetry: jam detection, stall tracking. */
public class IntakeTelemetry implements SubsystemTelemetry {
  private Intake intake; // Not final - can re-acquire if null
  private boolean subsystemAvailable = false;

  private static final double JAM_CURRENT_THRESHOLD_AMPS = 25.0;
  private static final double JAM_TIME_THRESHOLD_SECONDS = 0.25;

  // Jam detection
  private final Debouncer jamDebouncer =
      new Debouncer(JAM_TIME_THRESHOLD_SECONDS, Debouncer.DebounceType.kRising);
  private boolean jamDetected = false;
  private int totalJamCount = 0;

  private boolean running = false;
  private boolean wasRunning = false;
  private String direction = "STOPPED";
  private double appliedOutput = 0;
  private double currentAmps = 0;
  private double temperatureCelsius = 0;
  private double velocityRPM = 0;

  private double currentPerSpeedRatio = 0; // drag indicator

  // Device health — debounced to filter CAN bus transients
  private final Debouncer connectDebouncer =
      new Debouncer(DeviceHealthConstants.DISCONNECT_DEBOUNCE_SEC, Debouncer.DebounceType.kFalling);
  private boolean deviceConnected = false;
  private int deviceFaultsRaw = 0;

  private boolean stalled = false;
  private double stallStartTime = 0;
  private double stallDurationMs = 0;
  private boolean inStallCondition = false;

  public IntakeTelemetry() {
    this.intake = Intake.getInstance();
  }

  @Override
  public void update() {
    if (intake == null) {
      intake = Intake.getInstance();
    }

    if (intake == null) {
      subsystemAvailable = false;
      setDefaultValues();
      return;
    }

    subsystemAvailable = true;
    double now = Timer.getFPGATimestamp();

    try {
      appliedOutput = intake.getAppliedOutput();
      wasRunning = running;
      running = intake.isRunning();
      currentAmps = intake.getOutputCurrent();
      temperatureCelsius = intake.getTemperature();
      velocityRPM = intake.getVelocityRPM();

      // Device health
      deviceConnected = connectDebouncer.calculate(true);
      deviceFaultsRaw = intake.getStickyFaultsRaw();
    } catch (Throwable t) {
      deviceConnected = connectDebouncer.calculate(false);
      deviceFaultsRaw = -1;
      setDefaultValues();
      return;
    }

    boolean meetsStallCriteria =
        (currentAmps > StallDetectionConstants.INTAKE_STALL_CURRENT_AMPS)
            && (Math.abs(velocityRPM) < StallDetectionConstants.INTAKE_STALL_VELOCITY_RPM)
            && running; // Only check when commanded to run

    if (meetsStallCriteria) {
      if (!inStallCondition) {
        stallStartTime = now;
        inStallCondition = true;
      }
      stallDurationMs = (now - stallStartTime) * 1000.0;
      stalled = (stallDurationMs >= StallDetectionConstants.INTAKE_STALL_DEBOUNCE_MS);
    } else {
      inStallCondition = false;
      stallDurationMs = 0;
      stalled = false;
    }

    // Current-to-speed ratio (drag indicator)
    currentPerSpeedRatio = (Math.abs(velocityRPM) > 10) ? currentAmps / Math.abs(velocityRPM) : 0;

    // Direction
    if (appliedOutput > 0.05) {
      direction = "FORWARD";
    } else if (appliedOutput < -0.05) {
      direction = "REVERSE";
    } else {
      direction = "STOPPED";
    }

    // Cycle tracking: intake started (rising edge, forward direction)
    if (running && !wasRunning && appliedOutput > 0.05) {
      SafeLog.run(() -> CycleTracker.getInstance().intakeStarted());
    }

    // Jam detection using debouncer
    boolean highCurrent = running && (currentAmps > JAM_CURRENT_THRESHOLD_AMPS);
    boolean wasJammed = jamDetected;
    jamDetected = jamDebouncer.calculate(highCurrent);

    // Count on rising edge only
    if (jamDetected && !wasJammed) {
      totalJamCount++;
      SafeLog.run(() -> EventMarker.jamDetected("Intake"));
      SafeLog.run(() -> CycleTracker.getInstance().cycleAborted("Intake jam"));
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
    SafeLog.put("Intake/Available", subsystemAvailable);
    SafeLog.put("Intake/Running", running);
    SafeLog.put("Intake/Direction", direction);
    SafeLog.put("Intake/AppliedOutput", appliedOutput);
    SafeLog.put("Intake/CurrentAmps", currentAmps);
    SafeLog.put("Intake/TemperatureCelsius", temperatureCelsius);
    SafeLog.put("Intake/JamDetected", jamDetected);
    SafeLog.put("Intake/TotalJamCount", totalJamCount);
    SafeLog.put("Intake/CurrentPerSpeedRatio", currentPerSpeedRatio);

    // Device health
    SafeLog.put("Intake/Device/Connected", deviceConnected);
    SafeLog.put("Intake/Device/FaultsRaw", deviceFaultsRaw);

    SafeLog.put("Intake/VelocityRPM", velocityRPM);
    SafeLog.put("Intake/Stalled", stalled);
    SafeLog.put("Intake/StallDurationMs", stallDurationMs);
  }

  @Override
  public String getName() {
    return "Intake";
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
    jamDebouncer.calculate(false);
  }

  public boolean isDeviceConnected() {
    return deviceConnected;
  }

  public int getDeviceFaultsRaw() {
    return deviceFaultsRaw;
  }
}
