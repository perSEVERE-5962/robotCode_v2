package frc.robot.telemetry;

import frc.robot.commands.ShootOnTheMove;

/**
 * Logs ShootOnTheMove's intermediate calculations for post-match debugging. Reads static snapshot
 * fields from the command each cycle.
 */
public class ShotPredictorTelemetry implements SubsystemTelemetry {
  private boolean commandActive = false;
  private double distanceToHubM = 0;
  private double timeOfFlightSec = 0;
  private double compTargetX = 0;
  private double compTargetY = 0;
  private double driftX = 0;
  private double driftY = 0;
  private double computedRPM = 0;
  private double headingErrorRad = 0;
  private double headingSpeedRadPerSec = 0;

  @Override
  public void update() {
    try {
      commandActive = ShootOnTheMove.isActive();
      if (!commandActive) {
        setDefaultValues();
        return;
      }
      distanceToHubM = ShootOnTheMove.getSnapDistanceToHubM();
      timeOfFlightSec = ShootOnTheMove.getSnapTimeOfFlightSec();
      compTargetX = ShootOnTheMove.getSnapCompTargetX();
      compTargetY = ShootOnTheMove.getSnapCompTargetY();
      driftX = ShootOnTheMove.getSnapDriftX();
      driftY = ShootOnTheMove.getSnapDriftY();
      computedRPM = ShootOnTheMove.getSnapComputedRPM();
      headingErrorRad = ShootOnTheMove.getSnapHeadingErrorRad();
      headingSpeedRadPerSec = ShootOnTheMove.getSnapHeadingSpeedRadPerSec();
    } catch (Throwable t) {
      commandActive = false;
      setDefaultValues();
    }
  }

  @Override
  public void log() {
    SafeLog.put("Scoring/ShotPredictor/Active", commandActive);
    SafeLog.put("Scoring/ShotPredictor/DistanceToHubM", distanceToHubM);
    SafeLog.put("Scoring/ShotPredictor/TimeOfFlightSec", timeOfFlightSec);
    SafeLog.put("Scoring/ShotPredictor/CompTargetX", compTargetX);
    SafeLog.put("Scoring/ShotPredictor/CompTargetY", compTargetY);
    SafeLog.put("Scoring/ShotPredictor/DriftX", driftX);
    SafeLog.put("Scoring/ShotPredictor/DriftY", driftY);
    SafeLog.put("Scoring/ShotPredictor/ComputedRPM", computedRPM);
    SafeLog.put("Scoring/ShotPredictor/HeadingErrorRad", headingErrorRad);
    SafeLog.put("Scoring/ShotPredictor/HeadingSpeedRadPerSec", headingSpeedRadPerSec);
  }

  @Override
  public String getName() {
    return "ShotPredictor";
  }

  private void setDefaultValues() {
    distanceToHubM = 0;
    timeOfFlightSec = 0;
    compTargetX = 0;
    compTargetY = 0;
    driftX = 0;
    driftY = 0;
    computedRPM = 0;
    headingErrorRad = 0;
    headingSpeedRadPerSec = 0;
  }

  // Accessors for other telemetry classes
  public boolean isCommandActive() {
    return commandActive;
  }

  public double getComputedRPM() {
    return computedRPM;
  }

  public double getHeadingErrorRad() {
    return headingErrorRad;
  }
}
