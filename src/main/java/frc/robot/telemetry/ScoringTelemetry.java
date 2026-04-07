package frc.robot.telemetry;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.DeviceHealthConstants;
import frc.robot.util.HubShiftEngine;

/** Scoring readiness: composite ReadyToShoot from shooter + indexer + vision. */
public class ScoringTelemetry implements SubsystemTelemetry {
  private final ShooterTelemetry shooterTelemetry;
  private final IndexerTelemetry indexerTelemetry;
  private final VisionTelemetry visionTelemetry;

  // ReadyToShoot stays true through brief velocity dips during sustained fire.
  // Manual time-based debounce (not WPILib Debouncer,that has wrong initial baseline).
  private double readyFalseSince = 0;

  private boolean scoringAvailable = false;
  private boolean shooterReady = false;
  private boolean indexerClear = false;
  private boolean visionLocked = false;
  private boolean hasBall = true; // Stub: assume ball present until hopper sensor
  private boolean hubActive = true;
  private boolean readyToShoot = false;

  private boolean wonAuto = false;
  private boolean wonAutoFromFMS = false;
  private int hubShiftNumber = 0;
  private double timeToNextShiftSec = 0;

  private boolean previousShooterReady = false;
  private boolean previousIndexerClear = true;
  private boolean previousVisionLocked = false;
  private boolean previousHubActive = true;
  private boolean previousReadyToShoot = false;
  private boolean readyStateChanged = false;
  private String readyLostReason = "";

  private double readySinceTimestamp = 0;
  private double timeSinceReadyMs = 0;
  private boolean wasReady = false;

  public ScoringTelemetry(
      ShooterTelemetry shooterTelemetry,
      IndexerTelemetry indexerTelemetry,
      VisionTelemetry visionTelemetry) {
    this.shooterTelemetry = shooterTelemetry;
    this.indexerTelemetry = indexerTelemetry;
    this.visionTelemetry = visionTelemetry;
  }

  @Override
  public void update() {
    try {
      double now = Timer.getFPGATimestamp();

      if (shooterTelemetry == null || indexerTelemetry == null || visionTelemetry == null) {
        scoringAvailable = false;
        setDefaultValues();
        return;
      }

      scoringAvailable = true;

      previousShooterReady = shooterReady;
      previousIndexerClear = indexerClear;
      previousVisionLocked = visionLocked;
      previousHubActive = hubActive;
      previousReadyToShoot = readyToShoot;

      shooterReady = shooterTelemetry.isAtSpeed();
      indexerClear = !indexerTelemetry.isJamDetected();
      visionLocked = visionTelemetry.isLockedOnTarget();

      hasBall = true; // Stub: read from hopper sensor when available. Reset in setDefaultValues().

      // Hub state from HubShiftEngine (single source of truth for shift timing)
      HubShiftEngine hubEngine = HubShiftEngine.getInstance();
      HubShiftEngine.ShiftInfo hubInfo = hubEngine.getOfficialInfo();
      wonAuto = hubEngine.isWonAuto();
      wonAutoFromFMS = hubEngine.isWonAutoFromFMS();
      hubActive = hubInfo.hubActive();
      hubShiftNumber = hubInfo.phase().ordinal();
      timeToNextShiftSec = hubInfo.remainingInState();

      boolean rawReady = shooterReady && indexerClear && visionLocked && hasBall && hubActive;
      if (rawReady) {
        readyToShoot = true;
        readyFalseSince = 0;
      } else if (!readyToShoot) {
        readyFalseSince = 0;
      } else {
        // Was true, conditions now false,hold true during debounce window
        if (readyFalseSince == 0) readyFalseSince = now;
        if ((now - readyFalseSince) >= DeviceHealthConstants.READY_TO_SHOOT_DEBOUNCE_SEC) {
          readyToShoot = false;
        }
      }

      readyStateChanged = (readyToShoot != previousReadyToShoot);
      if (readyStateChanged && !readyToShoot) {
        // Ready just dropped: capture which conditions are currently failing
        readyLostReason = buildNotReadyReason();
      } else if (readyStateChanged && readyToShoot) {
        // Ready just gained: clear the reason
        readyLostReason = "";
      }
      // else: no change, preserve existing readyLostReason

      if (readyToShoot && !wasReady) {
        readySinceTimestamp = now;
      }
      if (readyToShoot) {
        timeSinceReadyMs = (now - readySinceTimestamp) * 1000.0;
      } else {
        timeSinceReadyMs = 0;
      }

      wasReady = readyToShoot;
    } catch (Throwable t) {
      scoringAvailable = false;
      setDefaultValues();
    }
  }

  private void setDefaultValues() {
    shooterReady = false;
    indexerClear = true; // Safe default: assume clear
    visionLocked = false;
    hasBall = false;
    hubActive = false;
    readyToShoot = false;
    timeSinceReadyMs = 0;
    hubShiftNumber = 0;
    timeToNextShiftSec = 0;
    readyStateChanged = false;
  }

  /** Build a string naming which subconditions are currently failing. */
  private String buildNotReadyReason() {
    StringBuilder sb = new StringBuilder();
    if (!shooterReady) {
      sb.append("Shooter");
    }
    if (!indexerClear) {
      if (sb.length() > 0) sb.append("+");
      sb.append("Indexer");
    }
    if (!visionLocked) {
      if (sb.length() > 0) sb.append("+");
      sb.append("Vision");
    }
    if (!hubActive) {
      if (sb.length() > 0) sb.append("+");
      sb.append("Hub");
    }
    return sb.length() > 0 ? sb.toString() : "Unknown";
  }

  @Override
  public void log() {
    SafeLog.put("Scoring/ReadyToShoot", readyToShoot);
    SafeLog.put("Scoring/Conditions/ShooterReady", shooterReady);
    SafeLog.put("Scoring/Conditions/VisionLocked", visionLocked);
    SafeLog.put("Scoring/Conditions/HasBall", hasBall);
    SafeLog.put("Scoring/Conditions/HubActive", hubActive);
    SafeLog.put("Scoring/Conditions/IndexerClear", indexerClear);
    SafeLog.put("Scoring/ReadyLostReason", readyLostReason);
    SafeLog.put("Scoring/Conditions/WonAuto", wonAuto);
    SafeLog.put("Scoring/ReadyStateChanged", readyStateChanged);
    SafeLog.put("Scoring/TimeSinceReadyMs", timeSinceReadyMs);

    if (Constants.TUNING_MODE) {
      SafeLog.put("Scoring/Available", scoringAvailable);
      SafeLog.put("Scoring/Previous/ShooterReady", previousShooterReady);
      SafeLog.put("Scoring/Previous/IndexerClear", previousIndexerClear);
      SafeLog.put("Scoring/Previous/VisionLocked", previousVisionLocked);
      SafeLog.put("Scoring/Previous/HubActive", previousHubActive);
      SafeLog.put("Scoring/Previous/ReadyToShoot", previousReadyToShoot);
      SafeLog.put("Scoring/HubShiftNumber", hubShiftNumber);
      SafeLog.put("Scoring/TimeToNextShiftSec", timeToNextShiftSec);
      SafeLog.put("Scoring/Conditions/WonAutoFromFMS", wonAutoFromFMS);
    }
  }

  @Override
  public String getName() {
    return "Scoring";
  }

  public boolean isReadyToShoot() {
    return readyToShoot;
  }

  public int getHubShiftNumber() {
    return hubShiftNumber;
  }

  public boolean isHubActive() {
    return hubActive;
  }

  public double getTimeToNextShiftSec() {
    return timeToNextShiftSec;
  }

  public boolean isReadyStateChanged() {
    return readyStateChanged;
  }

  public String getReadyLostReason() {
    return readyLostReason;
  }

  public boolean isWonAuto() {
    return wonAuto;
  }
}
