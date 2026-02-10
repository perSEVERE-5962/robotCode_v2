package frc.robot.telemetry;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.util.EventMarker;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Central orchestrator for all telemetry classes. Call updateAll() once per robotPeriodic() cycle.
 */
public class TelemetryManager {
  private static TelemetryManager instance;
  private final List<SubsystemTelemetry> telemetryList = new ArrayList<>();

  // Failure tracking for this cycle
  private int cycleFailures = 0;
  private String lastFailedName = "none";

  // Individual references for external access
  private ShooterTelemetry shooterTelemetry;
  private IndexerTelemetry indexerTelemetry;
  private IntakeTelemetry intakeTelemetry;
  private IntakeActuatorTelemetry intakeActuatorTelemetry;
  private HangerTelemetry hangerTelemetry;
  private VisionTelemetry visionTelemetry;
  private ScoringTelemetry scoringTelemetry;
  private DriveTelemetry driveTelemetry;
  private SystemHealthTelemetry systemHealthTelemetry;
  private CommandsTelemetry commandsTelemetry;
  private NetworkTelemetry networkTelemetry;
  private DriverInputTelemetry driverInputTelemetry;
  private MatchStatsTelemetry matchStatsTelemetry;
  private ShotVisualizerTelemetry shotVisualizerTelemetry;
  private ShotPredictorTelemetry shotPredictorTelemetry;

  public static TelemetryManager getInstance() {
    if (instance == null) {
      instance = new TelemetryManager();
    }
    return instance;
  }

  private TelemetryManager() {
    // Initialize telemetry classes (order matters for ScoringTelemetry)
    shooterTelemetry = new ShooterTelemetry();
    indexerTelemetry = new IndexerTelemetry();
    intakeTelemetry = new IntakeTelemetry();
    intakeActuatorTelemetry = new IntakeActuatorTelemetry();
    hangerTelemetry = new HangerTelemetry();
    visionTelemetry = new VisionTelemetry();
    scoringTelemetry = new ScoringTelemetry(shooterTelemetry, indexerTelemetry, visionTelemetry);
    driveTelemetry = new DriveTelemetry();
    systemHealthTelemetry = new SystemHealthTelemetry();
    commandsTelemetry = new CommandsTelemetry();
    networkTelemetry = new NetworkTelemetry();
    driverInputTelemetry = new DriverInputTelemetry();
    matchStatsTelemetry =
        new MatchStatsTelemetry(shooterTelemetry, visionTelemetry, scoringTelemetry);
    shotVisualizerTelemetry = new ShotVisualizerTelemetry(shooterTelemetry);
    shotPredictorTelemetry = new ShotPredictorTelemetry();

    // SystemHealth first (measures loop time), then Commands (callback-based)
    telemetryList.add(systemHealthTelemetry);
    telemetryList.add(commandsTelemetry);
    telemetryList.add(driveTelemetry);
    telemetryList.add(shooterTelemetry);
    telemetryList.add(indexerTelemetry);
    telemetryList.add(intakeTelemetry);
    telemetryList.add(intakeActuatorTelemetry);
    telemetryList.add(hangerTelemetry);
    telemetryList.add(visionTelemetry);
    telemetryList.add(scoringTelemetry);
    telemetryList.add(new MatchTelemetry());
    telemetryList.add(networkTelemetry);
    telemetryList.add(driverInputTelemetry);
    telemetryList.add(matchStatsTelemetry);
    telemetryList.add(shotVisualizerTelemetry);
    telemetryList.add(shotPredictorTelemetry);
  }

  /** Called from RobotContainer after vision init */
  public void setVision(Vision vision) {
    visionTelemetry.setVision(vision);
  }

  /** Called from RobotContainer after swerve init */
  public void setSwerveSubsystem(SwerveSubsystem swerveSubsystem) {
    driveTelemetry.setSwerveSubsystem(swerveSubsystem);
  }

  /** Called from RobotContainer after controller init */
  public void setControllers(XboxController driver, XboxController operator) {
    driverInputTelemetry.setControllers(driver, operator);
  }

  // =========================================================================
  // Safety Wrappers
  // =========================================================================

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

  /** Safe getName() that won't throw */
  private String safeGetName(SubsystemTelemetry telemetry) {
    try {
      return telemetry != null ? telemetry.getName() : "unknown";
    } catch (Throwable t) {
      return "unknown";
    }
  }

  // =========================================================================
  // Main Update Loop
  // =========================================================================

  /** Call once per robotPeriodic() - updates and logs all telemetry */
  public void updateAll() {
    cycleFailures = 0;
    lastFailedName = "none";

    for (SubsystemTelemetry telemetry : telemetryList) {
      String name = safeGetName(telemetry);
      runSafely(telemetry::update, name + "/update");
      runSafely(telemetry::log, name + "/log"); // Always run, even if update failed
    }

    // External utilities
    runSafely(() -> CycleTracker.getInstance().log(), "CycleTracker/log");
    runSafely(EventMarker::flushCycleEvents, "EventMarker/flush");

    // SafeLog's own health metrics
    SafeLog.logAndReset();

    // TelemetryManager health metrics
    runSafely(this::logHealth, "Health/Telemetry");
  }

  private void logHealth() {
    Logger.recordOutput("Health/Telemetry/Failures", cycleFailures);
    Logger.recordOutput("Health/Telemetry/LastFailed", lastFailedName);
  }

  // =========================================================================
  // Accessors (AlertManager / PostMatchSummary)
  // =========================================================================

  // Shooter
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

  // Indexer
  public double getIndexerTemperature() {
    return getSafely(() -> indexerTelemetry.getTemperature(), 0.0);
  }

  public boolean isIndexerJamDetected() {
    return getSafely(() -> indexerTelemetry.isJamDetected(), false);
  }

  public boolean isIndexerNotJammed() {
    return getSafely(() -> !indexerTelemetry.isJamDetected(), true);
  }

  public void clearIndexerJam() {
    runSafely(() -> indexerTelemetry.clearJam(), "Indexer/clearJam");
  }

  public int getIndexerJamCount() {
    return getSafely(() -> indexerTelemetry.getTotalJamCount(), 0);
  }

  // Intake
  public double getIntakeTemperature() {
    return getSafely(() -> intakeTelemetry.getTemperature(), 0.0);
  }

  public boolean isIntakeJamDetected() {
    return getSafely(() -> intakeTelemetry.isJamDetected(), false);
  }

  public void clearIntakeJam() {
    runSafely(() -> intakeTelemetry.clearJam(), "Intake/clearJam");
  }

  public int getIntakeJamCount() {
    return getSafely(() -> intakeTelemetry.getTotalJamCount(), 0);
  }

  // IntakeActuator
  public double getIntakeActuatorTemperature() {
    return getSafely(() -> intakeActuatorTelemetry.getTemperature(), 0.0);
  }

  // Scoring composite
  public boolean isReadyToShoot() {
    return getSafely(() -> scoringTelemetry.isReadyToShoot(), false);
  }

  // Vision
  public boolean isLockedOnTarget() {
    return getSafely(() -> visionTelemetry.isLockedOnTarget(), false);
  }

  // SystemHealth (for AlertManager)
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
    return getSafely(() -> intakeTelemetry.isStalled(), false);
  }

  // Network (for AlertManager - R704 compliance)
  public double getBandwidthPercent() {
    return getSafely(() -> networkTelemetry.getBandwidthPercent(), 0.0);
  }

  public boolean isBandwidthWarning() {
    return getSafely(() -> networkTelemetry.isWarning(), false);
  }

  public boolean isBandwidthCritical() {
    return getSafely(() -> networkTelemetry.isCritical(), false);
  }

  // SystemHealth - current draw (for PostMatchSummary)
  public double getTotalCurrentAmps() {
    return getSafely(() -> systemHealthTelemetry.getTotalCurrentAmps(), 0.0);
  }

  public double getPeakCurrentAmps() {
    return getSafely(() -> systemHealthTelemetry.getPeakCurrentAmps(), 0.0);
  }

  // Drive (for path following commands)
  public DriveTelemetry getDriveTelemetry() {
    return getSafely(() -> driveTelemetry, null);
  }
}
