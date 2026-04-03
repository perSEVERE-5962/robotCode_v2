package frc.robot.telemetry;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DeviceHealthConstants;
import frc.robot.Constants.StallDetectionConstants;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.util.EventMarker;
import frc.robot.util.JamProtection;

/** Intake telemetry: jam detection, stall tracking. */
public class IntakeTelemetry implements SubsystemTelemetry {
  private IntakeRoller intake; // grabbed again in update() if not ready yet
  private boolean subsystemAvailable = false;

  private static final double JAM_CURRENT_THRESHOLD_AMPS = 25.0;
  private static final double JAM_TIME_THRESHOLD_SECONDS = 0.25;

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

  private final Debouncer connectDebouncer =
      new Debouncer(DeviceHealthConstants.DISCONNECT_DEBOUNCE_SEC, Debouncer.DebounceType.kFalling);
  private boolean deviceConnected = false;
  private int deviceFaultsRaw = 0;

  private boolean stalled = false;
  private double stallStartTime = 0;
  private double stallDurationMs = 0;
  private boolean inStallCondition = false;

  private String activeCommandName = "none";

  private String jamProtectionState = "MONITORING";
  private int jamProtectionAttempts = 0;
  private boolean jamProtectionIntervening = false;

  public IntakeTelemetry() {
    this.intake = IntakeRoller.getInstance();
  }

  @Override
  public void update() {
    if (intake == null) {
      intake = IntakeRoller.getInstance();
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

    try {
      Command currentCmd = intake.getCurrentCommand();
      activeCommandName = (currentCmd != null) ? currentCmd.getName() : "none";
    } catch (Throwable t) {
      activeCommandName = "unknown";
    }

    currentPerSpeedRatio = (Math.abs(velocityRPM) > 10) ? currentAmps / Math.abs(velocityRPM) : 0;

    if (appliedOutput > 0.05) {
      direction = "FORWARD";
    } else if (appliedOutput < -0.05) {
      direction = "REVERSE";
    } else {
      direction = "STOPPED";
    }

    try {
      JamProtection jp = intake.getJamProtection();
      jamProtectionState = jp.getState().name();
      jamProtectionAttempts = jp.getReverseAttempts();
      jamProtectionIntervening = jp.isIntervening();
    } catch (Throwable t) {
      jamProtectionState = "UNKNOWN";
    }

    boolean highCurrent = running && (currentAmps > JAM_CURRENT_THRESHOLD_AMPS);
    boolean wasJammed = jamDetected;
    jamDetected = jamDebouncer.calculate(highCurrent);

    if (jamDetected && !wasJammed) {
      totalJamCount++;
      SafeLog.run(() -> EventMarker.jamDetected("Intake"));
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
  }

  @Override
  public void log() {
    SafeLog.put("Intake/Available", subsystemAvailable);
    SafeLog.put("Intake/Running", running);
    SafeLog.put("Intake/VelocityRPM", velocityRPM);
    SafeLog.put("Intake/CurrentAmps", currentAmps);
    SafeLog.put("Intake/Device/Connected", deviceConnected);
    SafeLog.put("Intake/Stalled", stalled);
    SafeLog.put("Intake/JamDetected", jamDetected);

    if (Constants.TUNING_MODE) {
      SafeLog.put("Intake/Direction", direction);
      SafeLog.put("Intake/AppliedOutput", appliedOutput);
      SafeLog.put("Intake/TemperatureCelsius", temperatureCelsius);
      SafeLog.put("Intake/TotalJamCount", totalJamCount);
      SafeLog.put("Intake/CurrentPerSpeedRatio", currentPerSpeedRatio);
      SafeLog.put("Intake/Device/FaultsRaw", deviceFaultsRaw);
      SafeLog.put("Intake/StallDurationMs", stallDurationMs);
      SafeLog.put("Intake/ActiveCommand", activeCommandName);
      SafeLog.put("Intake/JamProtection/State", jamProtectionState);
      SafeLog.put("Intake/JamProtection/Attempts", jamProtectionAttempts);
      SafeLog.put("Intake/JamProtection/Intervening", jamProtectionIntervening);
    }
  }

  @Override
  public String getName() {
    return "Intake";
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
}
