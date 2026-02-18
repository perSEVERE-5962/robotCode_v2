package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.telemetry.TelemetryManager;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Post-match health summary: battery, temps, overruns, ranked issues. */
public class PostMatchSummary {
  private static final PostMatchSummary instance = new PostMatchSummary();

  private boolean isTracking = false;
  private double trackingStartTime = 0;
  private double matchDurationSeconds = 0;

  private double minBatteryVoltage = Double.MAX_VALUE;
  private double maxMotorTempCelsius = 0;
  private int loopOverrunCount = 0;
  private int brownoutCount = 0;
  private int alertCount = 0;
  private int shotsFired = 0;
  private boolean wasBrownedOut = false;

  // Ranked issues: severity (higher = worse) + timestamp + description
  private final List<MatchIssue> issues = new ArrayList<>();
  private static final int MAX_ISSUES = 50;

  // Dedup: track which issues have already been recorded this match
  private boolean issuedBrownout = false;
  private boolean issuedLowBattery = false;
  private boolean issuedHighTemp = false;
  private int lastJamCount = 0;
  private int lastStallCount = 0;
  private double peakCurrentAmps = 0;

  private static final int BASE_SCORE = 100;
  private static final int BROWNOUT_PENALTY = 20;
  private static final int LOW_BATTERY_PENALTY = 10; // < 11V at any point
  private static final int HIGH_TEMP_PENALTY = 5; // > 65C
  private static final int OVERRUN_PENALTY = 1; // per 10 overruns

  private PostMatchSummary() {}

  private static class MatchIssue {
    final int severity; // higher = worse
    final double timestampSec;
    final String description;

    MatchIssue(int severity, double timestampSec, String description) {
      this.severity = severity;
      this.timestampSec = timestampSec;
      this.description = description;
    }
  }

  private void addIssue(int severity, String description) {
    if (issues.size() >= MAX_ISSUES) return;
    double elapsed = Timer.getFPGATimestamp() - trackingStartTime;
    issues.add(new MatchIssue(severity, elapsed, description));
  }

  public static PostMatchSummary getInstance() {
    return instance;
  }

  /** Start tracking match data. Call in autonomousInit() or teleopInit(). */
  public void startTracking() {
    if (isTracking) return; // Already tracking

    isTracking = true;
    trackingStartTime = Timer.getFPGATimestamp();

    minBatteryVoltage = RobotController.getBatteryVoltage();
    maxMotorTempCelsius = 0;
    loopOverrunCount = 0;
    brownoutCount = 0;
    alertCount = 0;
    shotsFired = 0;
    wasBrownedOut = false;
    peakCurrentAmps = 0;

    issues.clear();
    issuedBrownout = false;
    issuedLowBattery = false;
    issuedHighTemp = false;
    lastJamCount = 0;
    lastStallCount = 0;
  }

  /** Update tracking with current robot state. Call every cycle in robotPeriodic() when enabled. */
  public void updateTracking(double loopTimeMs) {
    if (!isTracking) return;

    double voltage = RobotController.getBatteryVoltage();
    if (voltage < minBatteryVoltage) {
      minBatteryVoltage = voltage;
    }

    try {
      double shooterTemp = 0, indexerTemp = 0, intakeTemp = 0;
      Shooter shooter = Shooter.getInstance();
      if (shooter != null) shooterTemp = shooter.getTemperature();
      Indexer indexer = Indexer.getInstance();
      if (indexer != null) indexerTemp = indexer.getTemperature();
      Intake intake = Intake.getInstance();
      if (intake != null) intakeTemp = intake.getTemperature();

      double currentMaxTemp = Math.max(shooterTemp, Math.max(indexerTemp, intakeTemp));
      if (currentMaxTemp > maxMotorTempCelsius) {
        maxMotorTempCelsius = currentMaxTemp;
      }
    } catch (Throwable t) {
    }

    if (loopTimeMs > 20.0) {
      loopOverrunCount++;
    }

    // Track brownouts (latch on brownout detection)
    boolean currentlyBrownedOut = RobotController.isBrownedOut();
    if (currentlyBrownedOut && !wasBrownedOut) {
      brownoutCount++;
      addIssue(100, String.format("Brownout at %.1fV", voltage));
    }
    wasBrownedOut = currentlyBrownedOut;

    // Track low battery as an issue (once)
    if (voltage < 11.0 && !issuedLowBattery) {
      issuedLowBattery = true;
      addIssue(60, String.format("Battery dropped to %.1fV", voltage));
    }

    // Track high temp as an issue (once per threshold crossing)
    if (maxMotorTempCelsius > 65.0 && !issuedHighTemp) {
      issuedHighTemp = true;
      addIssue(40, String.format("Motor temp reached %.0fC", maxMotorTempCelsius));
    }

    try {
      TelemetryManager tm = TelemetryManager.getInstance();
      int currentJams = tm.getIndexerJamCount() + tm.getIntakeJamCount();
      if (currentJams > lastJamCount) {
        addIssue(50, "Jam detected (total: " + currentJams + ")");
        lastJamCount = currentJams;
      }
    } catch (Throwable t) {
    }

    try {
      TelemetryManager tm = TelemetryManager.getInstance();
      int stalls = 0;
      if (tm.isShooterStalled()) stalls++;
      if (tm.isIndexerStalled()) stalls++;
      if (tm.isIntakeStalled()) stalls++;
      if (stalls > lastStallCount) {
        addIssue(70, "Motor stall detected (" + stalls + " motors)");
        lastStallCount = stalls;
      }
    } catch (Throwable t) {
    }

    try {
      double current = TelemetryManager.getInstance().getTotalCurrentAmps();
      if (current > peakCurrentAmps) {
        peakCurrentAmps = current;
      }
    } catch (Throwable t) {
    }

    try {
      alertCount = AlertManager.getInstance().getActiveAlertCount();
    } catch (Throwable t) {
    }
  }

  /** Stop tracking and generate summary. Call in disabledInit() after a match. */
  public void generateSummary() {
    if (!isTracking) return;

    isTracking = false;
    matchDurationSeconds = Timer.getFPGATimestamp() - trackingStartTime;

    shotsFired = TelemetryManager.getInstance().getTotalShots();
    int healthScore = calculateHealthScore();
    logSummary(healthScore);

    // Send notification (only for real matches or if duration > 30s)
    if (DriverStation.isFMSAttached() || matchDurationSeconds > 30) {
      sendNotification(healthScore);
    }
  }

  private int calculateHealthScore() {
    int score = BASE_SCORE;

    score -= brownoutCount * BROWNOUT_PENALTY;

    if (minBatteryVoltage < 11.0) {
      score -= LOW_BATTERY_PENALTY;
    }

    if (maxMotorTempCelsius > 65.0) {
      score -= HIGH_TEMP_PENALTY;
    }

    // Deduct for loop overruns (1 point per 10 overruns)
    score -= (loopOverrunCount / 10) * OVERRUN_PENALTY;

    return Math.max(0, Math.min(100, score));
  }

  private void logSummary(int healthScore) {
    Logger.recordOutput("Summary/MatchDurationSeconds", matchDurationSeconds);
    Logger.recordOutput("Summary/HealthScore", healthScore);
    Logger.recordOutput("Summary/MinBatteryVoltage", minBatteryVoltage);
    Logger.recordOutput("Summary/MaxMotorTempCelsius", maxMotorTempCelsius);
    Logger.recordOutput("Summary/LoopOverruns", loopOverrunCount);
    Logger.recordOutput("Summary/BrownoutCount", brownoutCount);
    Logger.recordOutput("Summary/AlertCount", alertCount);
    Logger.recordOutput("Summary/ShotsFired", shotsFired);
    Logger.recordOutput("Summary/PeakCurrentAmps", peakCurrentAmps);
    Logger.recordOutput("Summary/TotalIssueCount", issues.size());

    // Rank by severity descending, log top 3
    List<MatchIssue> ranked = new ArrayList<>(issues);
    ranked.sort(Comparator.comparingInt((MatchIssue i) -> i.severity).reversed());
    for (int i = 0; i < 3; i++) {
      String val =
          (i < ranked.size())
              ? String.format(
                  "[%d] %.1fs: %s",
                  ranked.get(i).severity, ranked.get(i).timestampSec, ranked.get(i).description)
              : "";
      Logger.recordOutput("Summary/TopIssue" + (i + 1), val);
    }
  }

  private void sendNotification(int healthScore) {
    String grade;
    if (healthScore >= 90) grade = "A";
    else if (healthScore >= 80) grade = "B";
    else if (healthScore >= 70) grade = "C";
    else if (healthScore >= 60) grade = "D";
    else grade = "F";

    StringBuilder sb = new StringBuilder();
    sb.append(
        String.format(
            "Health: %d/100 (%s)\nBattery: %.1fV | Temp: %.0fC | Overruns: %d",
            healthScore, grade, minBatteryVoltage, maxMotorTempCelsius, loopOverrunCount));

    if (!issues.isEmpty()) {
      List<MatchIssue> ranked = new ArrayList<>(issues);
      ranked.sort(Comparator.comparingInt((MatchIssue i) -> i.severity).reversed());
      sb.append("\n--- Top Issues ---");
      int limit = Math.min(3, ranked.size());
      for (int i = 0; i < limit; i++) {
        MatchIssue issue = ranked.get(i);
        sb.append(String.format("\n%d. [%.1fs] %s", i + 1, issue.timestampSec, issue.description));
      }
    }

    String message = sb.toString();
    if (healthScore >= 80) {
      ElasticUtil.sendInfo("Post-Match Summary", message, 8000);
    } else if (healthScore >= 60) {
      ElasticUtil.sendWarning("Post-Match Summary", message, 10000);
    } else {
      ElasticUtil.sendError("Post-Match Summary", message, 15000);
    }
  }

  public boolean isTracking() {
    return isTracking;
  }

  public int getLastHealthScore() {
    return calculateHealthScore();
  }

  public double getMinBatteryVoltage() {
    return minBatteryVoltage;
  }

  public double getMaxMotorTemp() {
    return maxMotorTempCelsius;
  }
}
