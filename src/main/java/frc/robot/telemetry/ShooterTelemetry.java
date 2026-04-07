package frc.robot.telemetry;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DeviceHealthConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StallDetectionConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.util.EventMarker;
import java.util.ArrayDeque;
import java.util.Deque;

/** Shooter telemetry: shot detection, spin-up tracking, fire rate. */
public class ShooterTelemetry implements SubsystemTelemetry {
  private Shooter shooter; // grabbed again in update() if not ready yet
  private boolean subsystemAvailable = false;

  private double spinUpStartTime = 0;
  private boolean wasSpinningUp = false;
  private double lastSpinUpDurationMs = -1; // -1 = not yet measured
  private boolean spinUpInterrupted = false;
  private boolean wasAtSpeed = false;
  private boolean hasReachedSpeed = false; // Latch: prevents oscillation from resetting timer
  private boolean wasCommanded = false;

  private double previousVelocityRPM = 0;
  private int totalShotCount = 0;

  private static final double FIRE_RATE_WINDOW_SEC = 2.0;
  private final Deque<Double> shotTimestamps = new ArrayDeque<>();

  private double shotDipTimestamp = 0;
  private boolean trackingRecovery = false;
  private double lastRecoveryMs = 0;

  private boolean wasAtSpeedBeforeShot = false;
  private boolean isRecoverySpinUp = false;

  private final LinearFilter spinUpFilter = LinearFilter.movingAverage(20);
  private double filteredSpinUpPercent = 0;

  private double velocityRPM = 0;
  private double targetRPM = 0;
  private double velocityError = 0;
  private double appliedOutput = 0;
  private double currentAmps = 0;
  private double temperatureCelsius = 0;
  private double busVoltage = 0;
  private boolean atSpeed = false;
  private double atSpeedPercent = 0;
  private boolean isSpinningUp = false;
  private boolean shotDetected = false;
  private double velocityDrop = 0;
  private double actualFireRate = 0;
  private double actualRecoveryMs = 0;

  private double lastShotVelocityRPM = 0;
  private boolean temperatureWarning = false;
  private boolean pidTuningEvent = false;
  private double prevKP = -1, prevKI = -1, prevKD = -1, prevFF = -1;

  private final Debouncer connectDebouncer =
      new Debouncer(DeviceHealthConstants.DISCONNECT_DEBOUNCE_SEC, Debouncer.DebounceType.kFalling);
  private boolean deviceConnected = false;
  private int deviceFaultsRaw = 0;

  private boolean stalled = false;
  private boolean wasStalled = false;
  private double stallStartTime = 0;
  private double stallDurationMs = 0;
  private boolean inStallCondition = false;

  private String shooterState = "IDLE";
  private String previousShooterState = "IDLE";
  private boolean stateChangedThisCycle = false;

  private String activeCommandName = "none";

  private double prevAppliedOutput = 0;
  private int torqueReversalsThisSec = 0;
  private int torqueReversalCount = 0; // rolling 1-second window
  private double torqueReversalWindowStart = 0;
  private boolean torqueReversalAlert = false;
  private static final int TORQUE_REVERSAL_ALERT_THRESHOLD = 5; // reversals/sec

  public ShooterTelemetry() {
    this.shooter = Shooter.getInstance();
  }

  @Override
  public void update() {
    if (shooter == null) {
      shooter = Shooter.getInstance();
    }

    if (shooter == null) {
      subsystemAvailable = false;
      setDefaultValues();
      return;
    }

    subsystemAvailable = true;
    double now = Timer.getFPGATimestamp();

    boolean wasAtSpeedBeforeUpdate = wasAtSpeed;

    try {
      velocityRPM = shooter.getVelocity();
      targetRPM = shooter.getTargetRPM();
      // Fallback: if desiredRPM is never set (Tuning4 commented out the setter),
      // use the dashboard TunableNumber as a reasonable approximation
      if (targetRPM == 0 && (Math.abs(appliedOutput) > 0.01 || Math.abs(velocityRPM) > 50)) {
        targetRPM = shooter.getTunableTargetRPM();
      }
      appliedOutput = shooter.getAppliedOutput();
      currentAmps = shooter.getOutputCurrent();
      temperatureCelsius = shooter.getTemperature();
      busVoltage = shooter.getBusVoltage();
      // Compute atSpeed ourselves because the no-arg isAtSpeed() checks against
      // desiredRPM which may not be set when moveToVelocityWithPID is overridden
      atSpeed = (targetRPM > 0) && (Math.abs(targetRPM - velocityRPM) < shooter.getToleranceRPM());

      deviceConnected = connectDebouncer.calculate(true);
      deviceFaultsRaw = shooter.getStickyFaultsRaw();
    } catch (Throwable t) {
      deviceConnected = connectDebouncer.calculate(false);
      deviceFaultsRaw = -1;
      setDefaultValues();
      return;
    }

    velocityError = targetRPM - velocityRPM;
    wasAtSpeed = atSpeed;

    atSpeedPercent = (targetRPM > 0) ? (velocityRPM / targetRPM) * 100.0 : 0;
    filteredSpinUpPercent =
        (targetRPM > 0) ? Math.max(0, Math.min(100, spinUpFilter.calculate(atSpeedPercent))) : 0;

    boolean meetsStallCriteria =
        (currentAmps > StallDetectionConstants.SHOOTER_STALL_CURRENT_AMPS)
            && (Math.abs(velocityRPM) < StallDetectionConstants.SHOOTER_STALL_VELOCITY_RPM)
            && (targetRPM > 0); // Only check when motor commanded to spin

    if (meetsStallCriteria) {
      if (!inStallCondition) {
        stallStartTime = now;
        inStallCondition = true;
      }
      stallDurationMs = (now - stallStartTime) * 1000.0;
      stalled = (stallDurationMs >= StallDetectionConstants.SHOOTER_STALL_DEBOUNCE_MS);
    } else {
      inStallCondition = false;
      stallDurationMs = 0;
      stalled = false;
    }

    wasStalled = stalled;

    boolean commanded = targetRPM > 0;
    wasCommanded = commanded;

    if (targetRPM <= 0) {
      hasReachedSpeed = false;
      if (trackingRecovery) {
        trackingRecovery = false;
      }
    }
    if (targetRPM > 0 && atSpeed) {
      hasReachedSpeed = true;
    }
    isSpinningUp = (targetRPM > 0) && !atSpeed && !hasReachedSpeed;
    if (isSpinningUp && !wasSpinningUp) {
      spinUpStartTime = now;
    }
    if (!isSpinningUp && wasSpinningUp) {
      if (atSpeed) {
        lastSpinUpDurationMs = (now - spinUpStartTime) * 1000.0;
        spinUpInterrupted = false;
      } else {
        // Spin-up interrupted - keep last successful time, just flag it
        spinUpInterrupted = true;
      }
    }
    wasSpinningUp = isSpinningUp;

    velocityDrop = previousVelocityRPM - velocityRPM;
    double shotDropThreshold = Shooter.getShotDropThreshold();
    shotDetected =
        (velocityDrop > shotDropThreshold)
            && (previousVelocityRPM > 1000.0)
            && wasAtSpeedBeforeUpdate
            && (targetRPM > 0);

    if (shotDetected) {
      totalShotCount++;
      lastShotVelocityRPM = previousVelocityRPM;
      shotTimestamps.addLast(now);
      shotDipTimestamp = now;
      trackingRecovery = true;
      wasAtSpeedBeforeShot = true;
      // Isolate external calls
      SafeLog.run(() -> EventMarker.shotFired(totalShotCount));
    }
    previousVelocityRPM = velocityRPM;

    temperatureWarning = temperatureCelsius > 65.0;

    pidTuningEvent = false;
    try {
      double curKP = Shooter.getTunableKP();
      double curKI = Shooter.getTunableKI();
      double curKD = Shooter.getTunableKD();
      double curFF = Shooter.getTunableFF();
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

    if (isSpinningUp) {
      isRecoverySpinUp = wasAtSpeedBeforeShot;
    } else if (atSpeed) {
      wasAtSpeedBeforeShot = false;
      isRecoverySpinUp = false;
    }

    while (!shotTimestamps.isEmpty() && (now - shotTimestamps.peekFirst()) > FIRE_RATE_WINDOW_SEC) {
      shotTimestamps.pollFirst();
    }
    actualFireRate = shotTimestamps.size() / FIRE_RATE_WINDOW_SEC;

    if (trackingRecovery && atSpeed) {
      lastRecoveryMs = (now - shotDipTimestamp) * 1000.0;
      trackingRecovery = false;
    }
    actualRecoveryMs = lastRecoveryMs;

    try {
      Command currentCmd = shooter.getCurrentCommand();
      activeCommandName = (currentCmd != null) ? currentCmd.getName() : "none";
    } catch (Throwable t) {
      activeCommandName = "unknown";
    }

    // Torque reversal detection: count sign-changes of appliedOutput per second
    // A healthy flywheel has 0 reversals. PID-to-zero on coasting flywheel = many reversals.
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

    previousShooterState = shooterState;
    shooterState = computeShooterState();
    stateChangedThisCycle = !shooterState.equals(previousShooterState);
  }

  private void setDefaultValues() {
    velocityRPM = 0;
    targetRPM = 0;
    velocityError = 0;
    appliedOutput = 0;
    currentAmps = 0;
    temperatureCelsius = 0;
    busVoltage = 0;
    atSpeed = false;
    atSpeedPercent = 0;
    filteredSpinUpPercent = 0;
    isSpinningUp = false;
    shotDetected = false;
    stalled = false;
    stallDurationMs = 0;
    inStallCondition = false;
    torqueReversalAlert = false;
    temperatureWarning = false;
  }

  /**
   * Derive a single state string from the existing boolean fields. Priority: STALLED > SHOT_FIRED >
   * RECOVERING > SPINNING_UP > AT_SPEED > RUNNING > IDLE.
   */
  private String computeShooterState() {
    if (stalled) return "STALLED";
    if (shotDetected) return "SHOT_FIRED";
    if (trackingRecovery && !atSpeed) return "RECOVERING";
    if (isSpinningUp) return "SPINNING_UP";
    if (atSpeed) return "AT_SPEED";
    if (targetRPM > 0) return "RUNNING";
    return "IDLE";
  }

  @Override
  public void log() {
    // Competition signals: always logged
    SafeLog.put("Shooter/Available", subsystemAvailable);
    SafeLog.put("Shooter/VelocityRPM", velocityRPM);
    SafeLog.put("Shooter/TargetRPM", targetRPM);
    SafeLog.put("Shooter/AtSpeed", atSpeed);
    SafeLog.put("Shooter/AppliedOutput", appliedOutput);
    SafeLog.put("Shooter/CurrentAmps", currentAmps);
    SafeLog.put("Shooter/TemperatureCelsius", temperatureCelsius);
    SafeLog.put("Shooter/TemperatureWarning", temperatureWarning);
    SafeLog.put("Shooter/BusVoltage", busVoltage);
    SafeLog.put("Shooter/ShotDetected", shotDetected);
    SafeLog.put("Shooter/TotalShots", totalShotCount);
    SafeLog.put("Shooter/LastShotVelocityRPM", lastShotVelocityRPM);
    SafeLog.put("Shooter/SpinUpTimeMs", lastSpinUpDurationMs);
    SafeLog.put("Shooter/Stalled", stalled);
    SafeLog.put("Shooter/Device/Connected", deviceConnected);
    SafeLog.put("Shooter/State", shooterState);

    // Debug/tuning signals: only logged when tuning to reduce CAN and log bandwidth
    if (Constants.TUNING_MODE) {
      SafeLog.put("Shooter/ActiveCommand", activeCommandName);
      SafeLog.put("Shooter/VelocityError", velocityError);
      SafeLog.put("Shooter/AtSpeedPercent", atSpeedPercent);
      SafeLog.put("Shooter/FilteredSpinUpPercent", filteredSpinUpPercent);
      SafeLog.put("Shooter/IsSpinningUp", isSpinningUp);
      SafeLog.put("Shooter/SpinUpInterrupted", spinUpInterrupted);
      SafeLog.put("Shooter/IsRecoverySpinUp", isRecoverySpinUp);
      SafeLog.put("Shooter/VelocityDrop", velocityDrop);
      SafeLog.put("Shooter/ActualFireRate", actualFireRate);
      SafeLog.put("Shooter/TargetFireRate", ShooterConstants.TARGET_FIRE_RATE_PER_SEC);
      SafeLog.put("Shooter/ActualRecoveryMs", actualRecoveryMs);
      SafeLog.put("Shooter/TargetRecoveryMs", ShooterConstants.TARGET_RECOVERY_MS);
      SafeLog.put("Shooter/PIDTuningEvent", pidTuningEvent);
      if (pidTuningEvent) {
        SafeLog.put("Shooter/Config/kP", prevKP);
        SafeLog.put("Shooter/Config/kI", prevKI);
        SafeLog.put("Shooter/Config/kD", prevKD);
        SafeLog.put("Shooter/Config/FF", prevFF);
      }
      SafeLog.put("Shooter/Device/FaultsRaw", deviceFaultsRaw);
      SafeLog.put("Shooter/StallDurationMs", stallDurationMs);
      SafeLog.put("Shooter/PreviousState", previousShooterState);
      SafeLog.put("Shooter/StateChanged", stateChangedThisCycle);
      SafeLog.put("Shooter/TorqueReversalsPerSec", torqueReversalCount);
      SafeLog.put("Shooter/TorqueReversalAlert", torqueReversalAlert);
    }
  }

  @Override
  public String getName() {
    return "Shooter";
  }

  public double getTemperature() {
    return temperatureCelsius;
  }

  public boolean isAtSpeed() {
    return atSpeed;
  }

  public int getTotalShots() {
    return totalShotCount;
  }

  public double getVelocityRPM() {
    return velocityRPM;
  }

  public boolean isStalled() {
    return stalled;
  }

  public boolean isDeviceConnected() {
    return deviceConnected;
  }

  public int getDeviceFaultsRaw() {
    return deviceFaultsRaw;
  }

  public double getAtSpeedPercent() {
    return atSpeedPercent;
  }

  public double getFilteredSpinUpPercent() {
    return filteredSpinUpPercent;
  }

  public boolean isSpinningUp() {
    return isSpinningUp;
  }

  public String getShooterState() {
    return shooterState;
  }

  public String getPreviousShooterState() {
    return previousShooterState;
  }

  public boolean isStateChanged() {
    return stateChangedThisCycle;
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
