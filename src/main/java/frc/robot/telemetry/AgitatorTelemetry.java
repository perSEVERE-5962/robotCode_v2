package frc.robot.telemetry;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DeviceHealthConstants;
import frc.robot.subsystems.Agitator;
import frc.robot.util.JamProtection;

/** Agitator telemetry: device health, stall detection, running state. */
public class AgitatorTelemetry implements SubsystemTelemetry {
  private Agitator agitator;
  private boolean subsystemAvailable = false;

  private static final double STALL_CURRENT_AMPS = 15.0;
  private static final double STALL_VELOCITY_RPM = 50.0;
  private static final double STALL_DEBOUNCE_MS = 200.0;

  private boolean running = false;
  private double appliedOutput = 0;
  private double currentAmps = 0;
  private double temperatureCelsius = 0;
  private double velocityRPM = 0;

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

  public AgitatorTelemetry() {
    this.agitator = Agitator.getInstance();
  }

  @Override
  public void update() {
    if (agitator == null) {
      agitator = Agitator.getInstance();
    }

    if (agitator == null) {
      subsystemAvailable = false;
      setDefaultValues();
      return;
    }

    subsystemAvailable = true;
    double now = Timer.getFPGATimestamp();

    try {
      appliedOutput = agitator.getAppliedOutput();
      running = agitator.isRunning();
      currentAmps = agitator.getOutputCurrent();
      temperatureCelsius = agitator.getTemperature();
      velocityRPM = agitator.getVelocityRPM();

      deviceConnected = connectDebouncer.calculate(true);
      deviceFaultsRaw = agitator.getStickyFaultsRaw();
    } catch (Throwable t) {
      deviceConnected = connectDebouncer.calculate(false);
      deviceFaultsRaw = -1;
      setDefaultValues();
      return;
    }

    boolean meetsStallCriteria =
        (currentAmps > STALL_CURRENT_AMPS)
            && (Math.abs(velocityRPM) < STALL_VELOCITY_RPM)
            && running;

    try {
      Command currentCmd = agitator.getCurrentCommand();
      activeCommandName = (currentCmd != null) ? currentCmd.getName() : "none";
    } catch (Throwable t) {
      activeCommandName = "unknown";
    }

    try {
      JamProtection jp = agitator.getJamProtection();
      jamProtectionState = jp.getState().name();
      jamProtectionAttempts = jp.getReverseAttempts();
      jamProtectionIntervening = jp.isIntervening();
    } catch (Throwable t) {
      jamProtectionState = "UNKNOWN";
    }

    if (meetsStallCriteria) {
      if (!inStallCondition) {
        stallStartTime = now;
        inStallCondition = true;
      }
      stallDurationMs = (now - stallStartTime) * 1000.0;
      stalled = (stallDurationMs >= STALL_DEBOUNCE_MS);
    } else {
      inStallCondition = false;
      stallDurationMs = 0;
      stalled = false;
    }
  }

  private void setDefaultValues() {
    running = false;
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
    SafeLog.put("Agitator/Available", subsystemAvailable);
    SafeLog.put("Agitator/Running", running);
    SafeLog.put("Agitator/AppliedOutput", appliedOutput);
    SafeLog.put("Agitator/CurrentAmps", currentAmps);
    SafeLog.put("Agitator/TemperatureCelsius", temperatureCelsius);
    SafeLog.put("Agitator/VelocityRPM", velocityRPM);
    SafeLog.put("Agitator/Device/Connected", deviceConnected);
    SafeLog.put("Agitator/Device/FaultsRaw", deviceFaultsRaw);
    SafeLog.put("Agitator/Stalled", stalled);
    SafeLog.put("Agitator/StallDurationMs", stallDurationMs);

    SafeLog.put("Agitator/ActiveCommand", activeCommandName);
    SafeLog.put("Agitator/JamProtection/State", jamProtectionState);
    SafeLog.put("Agitator/JamProtection/Attempts", jamProtectionAttempts);
    SafeLog.put("Agitator/JamProtection/Intervening", jamProtectionIntervening);
  }

  @Override
  public String getName() {
    return "Agitator";
  }

  public boolean isDeviceConnected() {
    return deviceConnected;
  }

  public int getDeviceFaultsRaw() {
    return deviceFaultsRaw;
  }

  public boolean isStalled() {
    return stalled;
  }

  public boolean isJamProtectionIntervening() {
    return jamProtectionIntervening;
  }

  public boolean isSubsystemAvailable() {
    return subsystemAvailable;
  }

  public double getTemperature() {
    return temperatureCelsius;
  }
}
