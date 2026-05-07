package frc.robot.telemetry;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.util.EventMarker;
import frc.robot.util.HubShiftEngine;
import frc.robot.util.SafeLog;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

/** Runs all telemetry updates each robotPeriodic() cycle. */
public class TelemetryManager {
  private static TelemetryManager instance;
  private final List<SubsystemTelemetry> telemetryList = new ArrayList<>();
  private final List<SubsystemTelemetry> tuningOnlyList = new ArrayList<>();

  private int cycleFailures = 0;
  private String lastFailedName = "none";

  // Signal staleness detector: tracks boolean signals that should not stay true indefinitely.
  // If a signal stays true longer than its threshold, log a warning.
  private final Map<String, StalenessTracker> stalenessTrackers = new LinkedHashMap<>();

  private static class StalenessTracker {
    final Supplier<Boolean> supplier;
    final double maxTrueSec;
    double trueStartTime = -1;
    boolean wasStale = false;

    StalenessTracker(Supplier<Boolean> supplier, double maxTrueSec) {
      this.supplier = supplier;
      this.maxTrueSec = maxTrueSec;
    }
  }

  private ShooterTelemetry shooterTelemetry;
  private IndexerTelemetry indexerTelemetry;
  private IntakeRollerTelemetry intakeRollerTelemetry;
  private IntakePivotTelemetry intakePivotTelemetry;
  private VisionTelemetry visionTelemetry;
  private ScoringTelemetry scoringTelemetry;
  private DriveTelemetry driveTelemetry;
  private SystemHealthTelemetry systemHealthTelemetry;
  private CommandsTelemetry commandsTelemetry;
  private MatchTelemetry matchTelemetry;
  private NetworkTelemetry networkTelemetry;
  private DriverInputTelemetry driverInputTelemetry;
  private MatchStatsTelemetry matchStatsTelemetry;
  private ShotVisualizerTelemetry shotVisualizerTelemetry;
  private ShotPredictorTelemetry shotPredictorTelemetry;
  private CANHealthTelemetry canHealthTelemetry;
  private DriverFeedbackTelemetry driverFeedbackTelemetry;
  private LEDTelemetry ledTelemetry;
  private AgitatorTelemetry agitatorTelemetry;

  public static TelemetryManager getInstance() {
    if (instance == null) {
      instance = new TelemetryManager();
    }
    return instance;
  }

  private TelemetryManager() {
    shooterTelemetry = new ShooterTelemetry();
    indexerTelemetry = new IndexerTelemetry();
    intakeRollerTelemetry = new IntakeRollerTelemetry();
    intakePivotTelemetry = new IntakePivotTelemetry();
    visionTelemetry = new VisionTelemetry();
    scoringTelemetry = new ScoringTelemetry(shooterTelemetry, indexerTelemetry, visionTelemetry);
    driveTelemetry = new DriveTelemetry();
    systemHealthTelemetry = new SystemHealthTelemetry();
    commandsTelemetry = new CommandsTelemetry();
    matchTelemetry = new MatchTelemetry();
    networkTelemetry = new NetworkTelemetry();
    driverInputTelemetry = new DriverInputTelemetry();
    matchStatsTelemetry =
        new MatchStatsTelemetry(shooterTelemetry, visionTelemetry, scoringTelemetry);
    shotVisualizerTelemetry = new ShotVisualizerTelemetry(shooterTelemetry);
    shotPredictorTelemetry = new ShotPredictorTelemetry();
    agitatorTelemetry = new AgitatorTelemetry();
    canHealthTelemetry =
        new CANHealthTelemetry(
            shooterTelemetry,
            indexerTelemetry,
            intakeRollerTelemetry,
            intakePivotTelemetry,
            agitatorTelemetry,
            visionTelemetry,
            driveTelemetry);
    driverFeedbackTelemetry = new DriverFeedbackTelemetry();
    ledTelemetry = new LEDTelemetry();

    // Competition: always registered
    telemetryList.add(systemHealthTelemetry);
    telemetryList.add(commandsTelemetry);
    telemetryList.add(driveTelemetry);
    telemetryList.add(shooterTelemetry);
    telemetryList.add(indexerTelemetry);
    telemetryList.add(intakeRollerTelemetry);
    telemetryList.add(intakePivotTelemetry);
    telemetryList.add(visionTelemetry);
    telemetryList.add(scoringTelemetry);
    telemetryList.add(matchTelemetry);
    telemetryList.add(networkTelemetry);
    telemetryList.add(driverInputTelemetry);
    telemetryList.add(canHealthTelemetry);
    telemetryList.add(driverFeedbackTelemetry);
    telemetryList.add(ledTelemetry);
    telemetryList.add(agitatorTelemetry);

    // Debug only: skipped in competition updateAll() to cut ~119 signals + LoggedTracer overhead
    tuningOnlyList.add(matchStatsTelemetry);
    tuningOnlyList.add(shotVisualizerTelemetry);
    tuningOnlyList.add(shotPredictorTelemetry);

    // Register signals that should not stay true for extended periods
    stalenessTrackers.put(
        "ShooterStalled", new StalenessTracker(() -> shooterTelemetry.isStalled(), 30.0));
    stalenessTrackers.put(
        "IndexerJam", new StalenessTracker(() -> indexerTelemetry.isJamDetected(), 60.0));
    stalenessTrackers.put(
        "IntakeJam", new StalenessTracker(() -> intakeRollerTelemetry.isJamDetected(), 60.0));
    stalenessTrackers.put(
        "IndexerStalled", new StalenessTracker(() -> indexerTelemetry.isStalled(), 30.0));
    stalenessTrackers.put(
        "IntakerStalled", new StalenessTracker(() -> intakeRollerTelemetry.isStalled(), 30.0));
  }

  /** Called from RobotContainer after vision init */
  public void setVision(Vision vision) {
    visionTelemetry.setVision(vision);
  }

  /** Called from RobotContainer after swerve init */
  public void setSwerveSubsystem(SwerveSubsystem swerveSubsystem) {
    driveTelemetry.setSwerveSubsystem(swerveSubsystem);
    shotVisualizerTelemetry.setSwerveSubsystem(swerveSubsystem);
  }

  /** Called from RobotContainer after controller init */
  public void setControllers(XboxController driver, XboxController operator) {
    driverInputTelemetry.setControllers(driver, operator);
  }

  /**
   * @return true if action succeeded
   */
  private boolean runSafely(Runnable action, String name) {
    try {
      action.run();
      return true;
    } catch (Throwable t) {
      cycleFailures++;
      lastFailedName = name;
      return false;
    }
  }

  /** Returns defaultValue on any failure. */
  private <T> T getSafely(Supplier<T> supplier, T defaultValue) {
    try {
      return supplier.get();
    } catch (Throwable t) {
      return defaultValue;
    }
  }

  private String safeGetName(SubsystemTelemetry telemetry) {
    try {
      return telemetry != null ? telemetry.getName() : "unknown";
    } catch (Throwable t) {
      return "unknown";
    }
  }

  /** Call once per robotPeriodic(). */
  public void updateAll() {
    cycleFailures = 0;
    lastFailedName = "none";

    for (SubsystemTelemetry telemetry : telemetryList) {
      String name = safeGetName(telemetry);
      if (Constants.TUNING_MODE) {
        double segStart = Timer.getFPGATimestamp();
        runSafely(telemetry::update, name + "/update");
        runSafely(telemetry::log, name + "/log");
        SafeLog.put(
            "LoggedTracer/Tel/" + name + "Ms", (Timer.getFPGATimestamp() - segStart) * 1000.0);
      } else {
        runSafely(telemetry::update, name + "/update");
        runSafely(telemetry::log, name + "/log");
      }
    }

    // Tuning-only telemetry: fully skipped in competition (no update, no log)
    if (Constants.TUNING_MODE) {
      for (SubsystemTelemetry telemetry : tuningOnlyList) {
        String name = safeGetName(telemetry);
        double segStart = Timer.getFPGATimestamp();
        runSafely(telemetry::update, name + "/update");
        runSafely(telemetry::log, name + "/log");
        SafeLog.put(
            "LoggedTracer/Tel/" + name + "Ms", (Timer.getFPGATimestamp() - segStart) * 1000.0);
      }
    }

    SafeLog.put("Config/TuningMode", Constants.TUNING_MODE);

    runSafely(EventMarker::flushCycleEvents, "EventMarker/flush");
    runSafely(this::checkStaleness, "Health/Staleness");

    SafeLog.logAndReset();

    runSafely(this::logHealth, "Health/Telemetry");
  }

  private void checkStaleness() {
    double now = Timer.getFPGATimestamp();
    for (Map.Entry<String, StalenessTracker> entry : stalenessTrackers.entrySet()) {
      StalenessTracker t = entry.getValue();
      boolean currentValue;
      try {
        currentValue = t.supplier.get();
      } catch (Throwable ex) {
        continue;
      }
      if (currentValue) {
        if (t.trueStartTime < 0) t.trueStartTime = now;
        boolean isStale = (now - t.trueStartTime) > t.maxTrueSec;
        if (isStale && !t.wasStale) {
          SafeLog.put("Health/Staleness/" + entry.getKey(), true);
          t.wasStale = true;
        }
      } else {
        t.trueStartTime = -1;
        if (t.wasStale) {
          SafeLog.put("Health/Staleness/" + entry.getKey(), false);
          t.wasStale = false;
        }
      }
    }
  }

  private void logHealth() {
    SafeLog.put("Health/Telemetry/Failures", cycleFailures);
    SafeLog.put("Health/Telemetry/LastFailed", lastFailedName);
  }

  public double getShooterTemperature() {
    return getSafely(() -> shooterTelemetry.getTemperature(), 0.0);
  }

  public boolean isShooterAtSpeed() {
    return getSafely(() -> shooterTelemetry.isAtSpeed(), false);
  }

  public int getTotalShots() {
    return getSafely(() -> shooterTelemetry.getTotalShots(), 0);
  }

  public double getShooterVelocityRPM() {
    return getSafely(() -> shooterTelemetry.getVelocityRPM(), 0.0);
  }

  public double getIndexerTemperature() {
    return getSafely(() -> indexerTelemetry.getTemperature(), 0.0);
  }

  public boolean isIndexerJamDetected() {
    return getSafely(() -> indexerTelemetry.isJamDetected(), false);
  }

  public boolean isIndexerNotJammed() {
    return getSafely(() -> !indexerTelemetry.isJamDetected(), true);
  }

  public int getIndexerJamCount() {
    return getSafely(() -> indexerTelemetry.getTotalJamCount(), 0);
  }

  public double getIntakeTemperature() {
    return getSafely(() -> intakeRollerTelemetry.getTemperature(), 0.0);
  }

  public boolean isIntakeJamDetected() {
    return getSafely(() -> intakeRollerTelemetry.isJamDetected(), false);
  }

  public int getIntakeJamCount() {
    return getSafely(() -> intakeRollerTelemetry.getTotalJamCount(), 0);
  }

  public double getIntakePivotTemperature() {
    return getSafely(() -> intakePivotTelemetry.getTemperature(), 0.0);
  }

  public boolean isAnyJamIntervening() {
    return getSafely(() -> intakeRollerTelemetry.isJamProtectionIntervening(), false)
        || getSafely(() -> indexerTelemetry.isJamProtectionIntervening(), false)
        || getSafely(() -> agitatorTelemetry.isJamProtectionIntervening(), false);
  }

  /** Returns which subsystem is currently jamming, or "none" if no jam. */
  public String getJamSource() {
    boolean intakeRoller =
        getSafely(() -> intakeRollerTelemetry.isJamProtectionIntervening(), false);
    boolean indexer = getSafely(() -> indexerTelemetry.isJamProtectionIntervening(), false);
    boolean agitator = getSafely(() -> agitatorTelemetry.isJamProtectionIntervening(), false);
    if (!intakeRoller && !indexer && !agitator) return "none";
    StringBuilder sb = new StringBuilder();
    if (intakeRoller) sb.append("IntakeRoller");
    if (indexer) {
      if (sb.length() > 0) sb.append("+");
      sb.append("Indexer");
    }
    if (agitator) {
      if (sb.length() > 0) sb.append("+");
      sb.append("Agitator");
    }
    return sb.toString();
  }

  public boolean isReadyToShoot() {
    return getSafely(() -> scoringTelemetry.isReadyToShoot(), false);
  }

  public boolean isLockedOnTarget() {
    return getSafely(() -> visionTelemetry.isLockedOnTarget(), false);
  }

  public double getPoseConfidence() {
    return getSafely(() -> visionTelemetry.getPoseConfidence(), 0.0);
  }

  public double getLoopTimeMs() {
    return getSafely(() -> systemHealthTelemetry.getLoopTimeMs(), 0.0);
  }

  public int getLoopOverrunCount() {
    return getSafely(() -> systemHealthTelemetry.getLoopOverrunCount(), 0);
  }

  public boolean isBrownoutRisk() {
    return getSafely(() -> systemHealthTelemetry.isBrownoutRisk(), false);
  }

  public int getBrownoutRiskLevel() {
    return getSafely(() -> systemHealthTelemetry.getBrownoutRiskLevel(), 0);
  }

  public boolean isShooterStalled() {
    return getSafely(() -> shooterTelemetry.isStalled(), false);
  }

  public boolean isIndexerStalled() {
    return getSafely(() -> indexerTelemetry.isStalled(), false);
  }

  public boolean isIntakeStalled() {
    return getSafely(() -> intakeRollerTelemetry.isStalled(), false);
  }

  public boolean isAgitatorStalled() {
    return getSafely(() -> agitatorTelemetry.isStalled(), false);
  }

  public double getBandwidthPercent() {
    return getSafely(() -> networkTelemetry.getBandwidthPercent(), 0.0);
  }

  public boolean isBandwidthWarning() {
    return getSafely(() -> networkTelemetry.isWarning(), false);
  }

  public boolean isBandwidthCritical() {
    return getSafely(() -> networkTelemetry.isCritical(), false);
  }

  public double getTotalCurrentAmps() {
    return getSafely(() -> systemHealthTelemetry.getTotalCurrentAmps(), 0.0);
  }

  public double getPeakCurrentAmps() {
    return getSafely(() -> systemHealthTelemetry.getPeakCurrentAmps(), 0.0);
  }

  public DriveTelemetry getDriveTelemetry() {
    return getSafely(() -> driveTelemetry, null);
  }

  public boolean isAllCANConnected() {
    return getSafely(() -> canHealthTelemetry.isAllConnected(), true);
  }

  public double getShooterAtSpeedPercent() {
    return getSafely(() -> shooterTelemetry.getAtSpeedPercent(), 0.0);
  }

  public boolean isShooterSpinningUp() {
    return getSafely(() -> shooterTelemetry.isSpinningUp(), false);
  }

  public double getFilteredSpinUpPercent() {
    return getSafely(() -> shooterTelemetry.getFilteredSpinUpPercent(), 0.0);
  }

  public double getAgitatorTemperature() {
    return getSafely(() -> agitatorTelemetry.getTemperature(), 0.0);
  }

  public double getBatteryVoltage() {
    return getSafely(() -> (double) RobotController.getBatteryVoltage(), 12.0);
  }

  public boolean isHubActive() {
    return getSafely(() -> HubShiftEngine.getInstance().getOfficialInfo().hubActive(), true);
  }

  public double getTimeToNextShiftSec() {
    return getSafely(() -> HubShiftEngine.getInstance().getOfficialInfo().remainingInState(), 0.0);
  }

  public boolean isWonAuto() {
    return getSafely(() -> HubShiftEngine.getInstance().isWonAuto(), false);
  }
}
