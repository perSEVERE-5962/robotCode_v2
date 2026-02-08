package frc.robot.telemetry;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.EventMarker;

/** Tracks intake-to-shot scoring cycles with per-phase timing. */
public class CycleTracker {
    private static CycleTracker instance;

    public enum CyclePhase {
        IDLE,
        INTAKING,
        INDEXING,
        AIMING,
        SHOOTING,
        RECOVERING
    }

    /** Robot intent for scoring zone */
    public enum ScoringZone {
        NONE,
        HUB_HIGH,   // Speaker/high goal
        HUB_LOW     // Amp/low goal
    }

    // Current state
    private CyclePhase currentPhase = CyclePhase.IDLE;
    private ScoringZone desiredScoringZone = ScoringZone.NONE;
    private double cycleStartTime = 0;
    private double phaseStartTime = 0;

    // Phase timestamps for timing
    private double intakeStartTime = 0;
    private double indexStartTime = 0;
    private double aimStartTime = 0;
    private double shootStartTime = 0;

    // Last cycle timing (ms)
    private double lastIntakeTimeMs = 0;
    private double lastIndexTimeMs = 0;
    private double lastAimTimeMs = 0;
    private double lastShootTimeMs = 0;
    private double lastTotalCycleTimeMs = 0;

    // Statistics
    private int completedCycles = 0;
    private int failedCycles = 0;
    private double totalCycleTimeMs = 0;
    private double totalIntakeTimeMs = 0;
    private double totalIndexTimeMs = 0;
    private double totalAimTimeMs = 0;
    private double totalShootTimeMs = 0;
    private double bestCycleTimeMs = Double.MAX_VALUE;
    private double worstCycleTimeMs = 0;

    private CycleTracker() {}

    public static CycleTracker getInstance() {
        if (instance == null) {
            instance = new CycleTracker();
        }
        return instance;
    }

    /** Call when intake starts acquiring a game piece */
    public void intakeStarted() {
        if (currentPhase == CyclePhase.IDLE || currentPhase == CyclePhase.RECOVERING) {
            double now = Timer.getFPGATimestamp();
            currentPhase = CyclePhase.INTAKING;
            cycleStartTime = now;
            intakeStartTime = now;
            phaseStartTime = now;
            EventMarker.mark(EventMarker.INTAKE, "Cycle started");
        }
    }

    /** Call when game piece is secured in indexer */
    public void intakeComplete() {
        if (currentPhase == CyclePhase.INTAKING) {
            double now = Timer.getFPGATimestamp();
            lastIntakeTimeMs = (now - intakeStartTime) * 1000;
            currentPhase = CyclePhase.INDEXING;
            indexStartTime = now;
            phaseStartTime = now;
            EventMarker.ballIntaked();
        }
    }

    /** Call when indexer has positioned game piece */
    public void indexComplete() {
        if (currentPhase == CyclePhase.INDEXING) {
            double now = Timer.getFPGATimestamp();
            lastIndexTimeMs = (now - indexStartTime) * 1000;
            currentPhase = CyclePhase.AIMING;
            aimStartTime = now;
            phaseStartTime = now;
        }
    }

    /** Call when vision has locked target and ready to shoot */
    public void aimComplete() {
        // Also accept INDEXING phase - auto-advance since indexing is typically very quick
        if (currentPhase == CyclePhase.INDEXING) {
            double now = Timer.getFPGATimestamp();
            lastIndexTimeMs = (now - indexStartTime) * 1000;
            lastAimTimeMs = 0;  // No dedicated aiming time
            currentPhase = CyclePhase.SHOOTING;
            shootStartTime = now;
            phaseStartTime = now;
        } else if (currentPhase == CyclePhase.AIMING) {
            double now = Timer.getFPGATimestamp();
            lastAimTimeMs = (now - aimStartTime) * 1000;
            currentPhase = CyclePhase.SHOOTING;
            shootStartTime = now;
            phaseStartTime = now;
        }
    }

    /** Call when shot is fired (detected by ShooterTelemetry) */
    public void shotFired() {
        if (currentPhase == CyclePhase.SHOOTING || currentPhase == CyclePhase.AIMING) {
            double now = Timer.getFPGATimestamp();

            // If shot without aim lock, record partial aim time
            if (currentPhase == CyclePhase.AIMING) {
                lastAimTimeMs = (now - aimStartTime) * 1000;
                shootStartTime = now;
            }

            lastShootTimeMs = (now - shootStartTime) * 1000;
            lastTotalCycleTimeMs = (now - cycleStartTime) * 1000;

            completedCycles++;
            totalCycleTimeMs += lastTotalCycleTimeMs;
            totalIntakeTimeMs += lastIntakeTimeMs;
            totalIndexTimeMs += lastIndexTimeMs;
            totalAimTimeMs += lastAimTimeMs;
            totalShootTimeMs += lastShootTimeMs;

            if (lastTotalCycleTimeMs < bestCycleTimeMs) {
                bestCycleTimeMs = lastTotalCycleTimeMs;
            }
            if (lastTotalCycleTimeMs > worstCycleTimeMs) {
                worstCycleTimeMs = lastTotalCycleTimeMs;
            }

            currentPhase = CyclePhase.RECOVERING;
            phaseStartTime = now;
        }
    }

    /** Call when shooter has recovered and ready for next cycle */
    public void recoveryComplete() {
        if (currentPhase == CyclePhase.RECOVERING) {
            currentPhase = CyclePhase.IDLE;
            phaseStartTime = Timer.getFPGATimestamp();
        }
    }

    /** Call if cycle is aborted (ball lost, jam, etc) */
    public void cycleAborted(String reason) {
        if (currentPhase != CyclePhase.IDLE) {
            failedCycles++;
            currentPhase = CyclePhase.IDLE;
            desiredScoringZone = ScoringZone.NONE;
            phaseStartTime = Timer.getFPGATimestamp();

            // M3: Reset all phase timestamps to prevent stale timing in next cycle
            cycleStartTime = 0;
            intakeStartTime = 0;
            indexStartTime = 0;
            aimStartTime = 0;
            shootStartTime = 0;

            SafeLog.run(() -> EventMarker.mark(EventMarker.ALERT, "Cycle aborted: " + reason));
        }
    }

    /** Set the desired scoring zone (call from shooting commands) */
    public void setDesiredScoringZone(ScoringZone zone) {
        this.desiredScoringZone = (zone != null) ? zone : ScoringZone.NONE;
    }

    /** Log all cycle metrics - called from TelemetryManager */
    public void log() {
        double now = Timer.getFPGATimestamp();
        double currentPhaseTimeMs = (now - phaseStartTime) * 1000;

        // Current state
        SafeLog.put("Cycle/Phase", currentPhase.name());
        SafeLog.put("Cycle/InProgress", currentPhase != CyclePhase.IDLE);
        SafeLog.put("Cycle/CurrentPhaseTimeMs", currentPhaseTimeMs);

        // Scoring intent
        SafeLog.put("Cycle/DesiredScoringZone", desiredScoringZone.name());

        // Last cycle breakdown
        SafeLog.put("Cycle/Last/IntakeMs", lastIntakeTimeMs);
        SafeLog.put("Cycle/Last/IndexMs", lastIndexTimeMs);
        SafeLog.put("Cycle/Last/AimMs", lastAimTimeMs);
        SafeLog.put("Cycle/Last/ShootMs", lastShootTimeMs);
        SafeLog.put("Cycle/Last/TotalMs", lastTotalCycleTimeMs);

        // Statistics
        SafeLog.put("Cycle/Stats/Completed", completedCycles);
        SafeLog.put("Cycle/Stats/Failed", failedCycles);
        SafeLog.put("Cycle/Stats/SuccessRate",
            completedCycles + failedCycles > 0
                ? (double) completedCycles / (completedCycles + failedCycles) * 100
                : 0);

        SafeLog.put("Cycle/Stats/AvgTotalMs",
            completedCycles > 0 ? totalCycleTimeMs / completedCycles : 0);
        SafeLog.put("Cycle/Stats/AvgIntakeMs",
            completedCycles > 0 ? totalIntakeTimeMs / completedCycles : 0);
        SafeLog.put("Cycle/Stats/AvgAimMs",
            completedCycles > 0 ? totalAimTimeMs / completedCycles : 0);
        SafeLog.put("Cycle/Stats/BestMs",
            bestCycleTimeMs == Double.MAX_VALUE ? 0 : bestCycleTimeMs);
        SafeLog.put("Cycle/Stats/WorstMs", worstCycleTimeMs);

        // Theoretical cycles per minute
        double avgCycleMs = completedCycles > 0 ? totalCycleTimeMs / completedCycles : 5000;
        if (avgCycleMs <= 0) avgCycleMs = 5000;
        SafeLog.put("Cycle/Stats/TheoreticalCPM", 60000 / avgCycleMs);
    }

    /** Reset for new match */
    public void reset() {
        currentPhase = CyclePhase.IDLE;
        desiredScoringZone = ScoringZone.NONE;
        completedCycles = 0;
        failedCycles = 0;
        totalCycleTimeMs = 0;
        totalIntakeTimeMs = 0;
        totalIndexTimeMs = 0;
        totalAimTimeMs = 0;
        totalShootTimeMs = 0;
        bestCycleTimeMs = Double.MAX_VALUE;
        worstCycleTimeMs = 0;
        lastIntakeTimeMs = 0;
        lastIndexTimeMs = 0;
        lastAimTimeMs = 0;
        lastShootTimeMs = 0;
        lastTotalCycleTimeMs = 0;
    }

    // Accessors
    public CyclePhase getCurrentPhase() { return currentPhase; }
    public int getCompletedCycles() { return completedCycles; }
    public int getFailedCycles() { return failedCycles; }
    public double getLastTotalCycleTimeMs() { return lastTotalCycleTimeMs; }
}
