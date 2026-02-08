package frc.robot.telemetry;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/** Match-level stats: shots by phase, time allocation, efficiency. */
public class MatchStatsTelemetry implements SubsystemTelemetry {
    private final ShooterTelemetry shooterTelemetry;
    private final VisionTelemetry visionTelemetry;

    // Match phase tracking
    private boolean wasEnabled = false;
    private double matchStartTime = 0;
    private double endgameStartTime = 0;
    private boolean inEndgame = false;

    // Scoring by phase
    private int autoShots = 0;
    private int teleopShots = 0;
    private int endgameShots = 0;
    private int lastTotalShots = 0;

    // Time allocation (ms)
    private double timeShootingMs = 0;
    private double timeIntakingMs = 0;
    private double timeAimingMs = 0;
    private double timeIdleMs = 0;

    // Efficiency metrics
    private double shooterUptimeMs = 0;
    private double visionLockTimeMs = 0;

    // Tracking state
    private double lastUpdateTime = 0;

    public MatchStatsTelemetry(ShooterTelemetry shooter, VisionTelemetry vision) {
        this.shooterTelemetry = shooter;
        this.visionTelemetry = vision;
    }

    @Override
    public void update() {
        double now = Timer.getFPGATimestamp();

        // L1 fix: First-frame guard for dt calculation
        double dt = (lastUpdateTime > 0) ? (now - lastUpdateTime) : 0.02;
        lastUpdateTime = now;

        boolean currentlyEnabled = DriverStation.isEnabled();

        // C4 fix: Detect match start BEFORE early return, update wasEnabled unconditionally
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
        if (currentTotalShots > lastTotalShots) {
            int newShots = currentTotalShots - lastTotalShots;
            if (DriverStation.isAutonomous()) {
                autoShots += newShots;
            } else if (inEndgame) {
                endgameShots += newShots;
            } else {
                teleopShots += newShots;
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

        // Track time allocation based on cycle phase
        // H2 fix: Handle all phases including INDEXING and RECOVERING
        CycleTracker.CyclePhase phase = CycleTracker.getInstance().getCurrentPhase();
        double dtMs = dt * 1000;
        switch (phase) {
            case INTAKING -> timeIntakingMs += dtMs;
            case INDEXING -> timeIntakingMs += dtMs;  // count as intake time
            case AIMING -> timeAimingMs += dtMs;
            case SHOOTING -> timeShootingMs += dtMs;
            case RECOVERING -> timeShootingMs += dtMs;  // count as shooting time
            case IDLE -> timeIdleMs += dtMs;
        }
    }

    @Override
    public void log() {
        // Shots by phase
        int totalShots = autoShots + teleopShots + endgameShots;
        SafeLog.put("MatchStats/AutoShots", autoShots);
        SafeLog.put("MatchStats/TeleopShots", teleopShots);
        SafeLog.put("MatchStats/EndgameShots", endgameShots);
        SafeLog.put("MatchStats/TotalShots", totalShots);

        // Cycles (from CycleTracker)
        CycleTracker tracker = CycleTracker.getInstance();
        SafeLog.put("MatchStats/CompletedCycles", tracker.getCompletedCycles());
        SafeLog.put("MatchStats/FailedCycles", tracker.getFailedCycles());

        // Time allocation (percentages)
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

        // Rates
        double matchDurationMin = (Timer.getFPGATimestamp() - matchStartTime) / 60.0;
        if (matchDurationMin > 0.05) { // Avoid divide by zero at start
            SafeLog.put("MatchStats/ShotsPerMinute", totalShots / matchDurationMin);
            SafeLog.put("MatchStats/CyclesPerMinute", tracker.getCompletedCycles() / matchDurationMin);
        } else {
            SafeLog.put("MatchStats/ShotsPerMinute", 0.0);
            SafeLog.put("MatchStats/CyclesPerMinute", 0.0);
        }
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
    }
}
