package frc.robot.telemetry;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/** Match-level stats: shots by phase, time allocation, efficiency. */
public class MatchStatsTelemetry implements SubsystemTelemetry {
  private final ShooterTelemetry shooterTelemetry;
  private final VisionTelemetry visionTelemetry;
  private final ScoringTelemetry scoringTelemetry; // nullable

  private boolean wasEnabled = false;
  private double matchStartTime = 0;
  private double endgameStartTime = 0;
  private boolean inEndgame = false;

  private int autoShots = 0;
  private int teleopShots = 0;
  private int endgameShots = 0;
  private int lastTotalShots = 0;

  private double timeShootingMs = 0;
  private double timeIntakingMs = 0;
  private double timeAimingMs = 0;
  private double timeIdleMs = 0;

  private double shooterUptimeMs = 0;
  private double visionLockTimeMs = 0;

  private int shotsShift1 = 0;
  private int shotsShift2 = 0;
  private int shotsShift3 = 0;
  private int shotsShift4 = 0;
  private int shotsShiftEndgame = 0;

  private int activeHubShots = 0;
  private int inactiveHubShots = 0;

  private double activeHubTimeMs = 0;
  private double firingDuringActiveMs = 0;

  private double lastUpdateTime = 0;

  public MatchStatsTelemetry(
      ShooterTelemetry shooter, VisionTelemetry vision, ScoringTelemetry scoring) {
    this.shooterTelemetry = shooter;
    this.visionTelemetry = vision;
    this.scoringTelemetry = scoring;
  }

  @Override
  public void update() {
    double now = Timer.getFPGATimestamp();

    double dt = (lastUpdateTime > 0) ? (now - lastUpdateTime) : 0.02;
    lastUpdateTime = now;

    boolean currentlyEnabled = DriverStation.isEnabled();

    if (!wasEnabled && currentlyEnabled) {
      matchStartTime = now;
      // Reset endgame tracking for new match
      inEndgame = false;
    }
    wasEnabled = currentlyEnabled;

    if (!currentlyEnabled) {
      return;
    }

    // Detect endgame (last 30 seconds of teleop)
    if (DriverStation.isTeleop() && !inEndgame) {
      double matchTime = DriverStation.getMatchTime();
      if (matchTime <= 30 && matchTime > 0) {
        endgameStartTime = now;
        inEndgame = true;
      }
    }

    // Track shots by phase
    int currentTotalShots = shooterTelemetry.getTotalShots();
    int newShots = currentTotalShots - lastTotalShots;
    if (newShots > 0) {
      if (DriverStation.isAutonomous()) {
        autoShots += newShots;
      } else if (inEndgame) {
        endgameShots += newShots;
      } else {
        teleopShots += newShots;
      }

      // Per-shift shot tracking (REBUILT hub strategy)
      if (scoringTelemetry != null) {
        int shift = scoringTelemetry.getHubShiftNumber();
        boolean hubActive = scoringTelemetry.isHubActive();
        switch (shift) {
          case 1 -> shotsShift1 += newShots;
          case 2 -> shotsShift2 += newShots;
          case 3 -> shotsShift3 += newShots;
          case 4 -> shotsShift4 += newShots;
          case 0 -> shotsShiftEndgame += newShots;
          default -> {} // auto or unexpected shift value
        }
        if (hubActive) activeHubShots += newShots;
        else inactiveHubShots += newShots;
      }
    }
    lastTotalShots = currentTotalShots;

    // Track shooter uptime
    if (shooterTelemetry.isAtSpeed()) {
      shooterUptimeMs += dt * 1000;
    }

    // Track vision lock time
    if (visionTelemetry.isLockedOnTarget()) {
      visionLockTimeMs += dt * 1000;
    }

    // Track time allocation based on subsystem activity
    double dtMs = dt * 1000;
    if (shooterTelemetry.isAtSpeed()) {
      timeShootingMs += dtMs;
    } else if (shooterTelemetry.isSpinningUp()) {
      timeAimingMs += dtMs;
    } else {
      timeIdleMs += dtMs;
    }

    // Hub utilization: time at-speed while our hub is active
    if (scoringTelemetry != null && DriverStation.isTeleop()) {
      if (scoringTelemetry.isHubActive()) {
        activeHubTimeMs += dtMs;
        if (shooterTelemetry.isAtSpeed()) {
          firingDuringActiveMs += dtMs;
        }
      }
    }
  }

  @Override
  public void log() {
    int totalShots = autoShots + teleopShots + endgameShots;
    SafeLog.put("MatchStats/AutoShots", autoShots);
    SafeLog.put("MatchStats/TeleopShots", teleopShots);
    SafeLog.put("MatchStats/EndgameShots", endgameShots);
    SafeLog.put("MatchStats/TotalShots", totalShots);

    double totalTime = timeShootingMs + timeIntakingMs + timeAimingMs + timeIdleMs;
    if (totalTime > 0) {
      SafeLog.put("MatchStats/Time/ShootingPct", timeShootingMs / totalTime * 100);
      SafeLog.put("MatchStats/Time/IntakingPct", timeIntakingMs / totalTime * 100);
      SafeLog.put("MatchStats/Time/AimingPct", timeAimingMs / totalTime * 100);
      SafeLog.put("MatchStats/Time/IdlePct", timeIdleMs / totalTime * 100);
    } else {
      SafeLog.put("MatchStats/Time/ShootingPct", 0.0);
      SafeLog.put("MatchStats/Time/IntakingPct", 0.0);
      SafeLog.put("MatchStats/Time/AimingPct", 0.0);
      SafeLog.put("MatchStats/Time/IdlePct", 0.0);
    }

    // Efficiency metrics
    SafeLog.put("MatchStats/ShooterUptimeMs", shooterUptimeMs);
    SafeLog.put("MatchStats/VisionLockTimeMs", visionLockTimeMs);

    double matchDurationMin = (Timer.getFPGATimestamp() - matchStartTime) / 60.0;
    if (matchDurationMin > 0.05) { // Avoid divide by zero at start
      SafeLog.put("MatchStats/ShotsPerMinute", totalShots / matchDurationMin);
    } else {
      SafeLog.put("MatchStats/ShotsPerMinute", 0.0);
    }

    SafeLog.put("MatchStats/ShotsInShift/1", shotsShift1);
    SafeLog.put("MatchStats/ShotsInShift/2", shotsShift2);
    SafeLog.put("MatchStats/ShotsInShift/3", shotsShift3);
    SafeLog.put("MatchStats/ShotsInShift/4", shotsShift4);
    SafeLog.put("MatchStats/ShotsInShift/Endgame", shotsShiftEndgame);
    SafeLog.put("MatchStats/ActiveHubShots", activeHubShots);
    SafeLog.put("MatchStats/InactiveHubShots", inactiveHubShots);
    double hubUtil = activeHubTimeMs > 0 ? (firingDuringActiveMs / activeHubTimeMs) * 100 : 0;
    SafeLog.put("MatchStats/HubUtilization", hubUtil);
  }

  @Override
  public String getName() {
    return "MatchStats";
  }

  /** Reset for new match */
  public void reset() {
    autoShots = 0;
    teleopShots = 0;
    endgameShots = 0;
    lastTotalShots = 0;
    timeShootingMs = 0;
    timeIntakingMs = 0;
    timeAimingMs = 0;
    timeIdleMs = 0;
    shooterUptimeMs = 0;
    visionLockTimeMs = 0;
    matchStartTime = 0;
    endgameStartTime = 0;
    inEndgame = false;
    wasEnabled = false;
    shotsShift1 = 0;
    shotsShift2 = 0;
    shotsShift3 = 0;
    shotsShift4 = 0;
    shotsShiftEndgame = 0;
    activeHubShots = 0;
    inactiveHubShots = 0;
    activeHubTimeMs = 0;
    firingDuringActiveMs = 0;
  }
}
