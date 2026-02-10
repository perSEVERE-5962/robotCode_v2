package frc.robot.telemetry;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DeviceHealthConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StallDetectionConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.util.EventMarker;
import java.util.ArrayDeque;
import java.util.Deque;

/** Shooter telemetry: shot detection, spin-up tracking, fire rate. */
public class ShooterTelemetry implements SubsystemTelemetry {
  private Shooter shooter; // Not final - can re-acquire if null
  private boolean subsystemAvailable = false;

  // Spin-up tracking
  private double spinUpStartTime = 0;
  private boolean wasSpinningUp = false;
  private double lastSpinUpDurationMs = 0;
  private boolean spinUpInterrupted = false;
  private boolean wasAtSpeed = false;

  // Shot detection
  private double previousVelocityRPM = 0;
  private int totalShotCount = 0;

  // Fire rate calculation (rolling window)
  private static final double FIRE_RATE_WINDOW_SEC = 2.0;
  private final Deque<Double> shotTimestamps = new ArrayDeque<>();

  // Recovery time tracking
  private double shotDipTimestamp = 0;
  private boolean trackingRecovery = false;
  private double lastRecoveryMs = 0;

  // Recovery vs cold-start classification
  private boolean wasAtSpeedBeforeShot = false;
  private boolean isRecoverySpinUp = false;

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

  // Device health — debounced to filter CAN bus transients
  private final Debouncer connectDebouncer =
      new Debouncer(DeviceHealthConstants.DISCONNECT_DEBOUNCE_SEC, Debouncer.DebounceType.kFalling);
  private boolean deviceConnected = false;
  private int deviceFaultsRaw = 0;

  private boolean stalled = false;
  private double stallStartTime = 0;
  private double stallDurationMs = 0;
  private boolean inStallCondition = false; // Current cycle meets stall criteria

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

    // Capture previous atSpeed state BEFORE updating (critical for shot detection)
    boolean wasAtSpeedBeforeUpdate = wasAtSpeed;

    try {
      velocityRPM = shooter.getVelocityRPM();
      targetRPM = shooter.getTargetRPM();
      appliedOutput = shooter.getAppliedOutput();
      currentAmps = shooter.getOutputCurrent();
      temperatureCelsius = shooter.getTemperature();
      busVoltage = shooter.getBusVoltage();
      atSpeed = shooter.isAtSpeed();

      // Device health
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

    // Spin-up tracking
    isSpinningUp = (targetRPM > 0) && !atSpeed;
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

    // Shot detection: velocity drop while previously at speed and still commanded to spin
    velocityDrop = previousVelocityRPM - velocityRPM;
    double shotDropThreshold = shooter.getShotDropThreshold();
    shotDetected =
        (velocityDrop > shotDropThreshold)
            && (previousVelocityRPM > ShooterConstants.SHOT_DETECTION_MIN_RPM)
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
      SafeLog.run(() -> CycleTracker.getInstance().shotFired());
    }
    previousVelocityRPM = velocityRPM;

    temperatureWarning = temperatureCelsius > ShooterConstants.TEMP_WARNING_CELSIUS;

    // PID audit trail: detect when tunable gains change
    pidTuningEvent = false;
    try {
      double curKP = shooter.getTunableKP();
      double curKI = shooter.getTunableKI();
      double curKD = shooter.getTunableKD();
      double curFF = shooter.getTunableFF();
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

    // Classify spin-up type: recovery (after shot) vs cold start (from zero)
    if (isSpinningUp) {
      isRecoverySpinUp = wasAtSpeedBeforeShot;
    } else if (atSpeed) {
      wasAtSpeedBeforeShot = false;
      isRecoverySpinUp = false;
    }

    // Fire rate calculation (rolling window)
    while (!shotTimestamps.isEmpty() && (now - shotTimestamps.peekFirst()) > FIRE_RATE_WINDOW_SEC) {
      shotTimestamps.pollFirst();
    }
    actualFireRate = shotTimestamps.size() / FIRE_RATE_WINDOW_SEC;

    // Recovery time tracking
    if (trackingRecovery && atSpeed) {
      lastRecoveryMs = (now - shotDipTimestamp) * 1000.0;
      trackingRecovery = false;
      // Isolate external call
      SafeLog.run(() -> CycleTracker.getInstance().recoveryComplete());
    }
    actualRecoveryMs = lastRecoveryMs;
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
    isSpinningUp = false;
    shotDetected = false;
    stalled = false;
    stallDurationMs = 0;
    inStallCondition = false;
  }

  @Override
  public void log() {
    SafeLog.put("Shooter/Available", subsystemAvailable);
    SafeLog.put("Shooter/VelocityRPM", velocityRPM);
    SafeLog.put("Shooter/TargetRPM", targetRPM);
    SafeLog.put("Shooter/VelocityError", velocityError);
    SafeLog.put("Shooter/AtSpeed", atSpeed);
    SafeLog.put("Shooter/AtSpeedPercent", atSpeedPercent);
    SafeLog.put("Shooter/AppliedOutput", appliedOutput);
    SafeLog.put("Shooter/CurrentAmps", currentAmps);
    SafeLog.put("Shooter/TemperatureCelsius", temperatureCelsius);
    SafeLog.put("Shooter/BusVoltage", busVoltage);
    SafeLog.put("Shooter/IsSpinningUp", isSpinningUp);
    SafeLog.put("Shooter/SpinUpTimeMs", lastSpinUpDurationMs);
    SafeLog.put("Shooter/SpinUpInterrupted", spinUpInterrupted);
    SafeLog.put("Shooter/ShotDetected", shotDetected);
    SafeLog.put("Shooter/TotalShots", totalShotCount);
    SafeLog.put("Shooter/VelocityDrop", velocityDrop);
    SafeLog.put("Shooter/ActualFireRate", actualFireRate);
    SafeLog.put("Shooter/TargetFireRate", ShooterConstants.TARGET_FIRE_RATE_PER_SEC);
    SafeLog.put("Shooter/ActualRecoveryMs", actualRecoveryMs);
    SafeLog.put("Shooter/TargetRecoveryMs", ShooterConstants.TARGET_RECOVERY_MS);
    SafeLog.put("Shooter/IsRecoverySpinUp", isRecoverySpinUp);

    SafeLog.put("Shooter/LastShotVelocityRPM", lastShotVelocityRPM);
    SafeLog.put("Shooter/TemperatureWarning", temperatureWarning);
    SafeLog.put("Shooter/PIDTuningEvent", pidTuningEvent);
    if (pidTuningEvent) {
      SafeLog.put("Shooter/Config/kP", prevKP);
      SafeLog.put("Shooter/Config/kI", prevKI);
      SafeLog.put("Shooter/Config/kD", prevKD);
      SafeLog.put("Shooter/Config/FF", prevFF);
    }

    // Device health
    SafeLog.put("Shooter/Device/Connected", deviceConnected);
    SafeLog.put("Shooter/Device/FaultsRaw", deviceFaultsRaw);

    SafeLog.put("Shooter/Stalled", stalled);
    SafeLog.put("Shooter/StallDurationMs", stallDurationMs);
  }

  @Override
  public String getName() {
    return "Shooter";
  }

  // Accessors for TelemetryManager
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
}
