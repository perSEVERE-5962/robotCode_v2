package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.DeviceHealthConstants;
import frc.robot.Constants.SpatialLaunchConstants;

/**
 * Scoring readiness composite with per-component debounce and heading hysteresis. Gates
 * ReadyToShoot on: shooterReady, indexerClear, visionLocked, hasBall, fireAuthorized,
 * shotConfident, headingOnTarget, !inExclusionZone. Currently shotConfident, hasBall, and
 * headingOnTarget pass through as stubs until their data sources (ShotCalculator, hopper sensor)
 * are wired.
 *
 * <p>Hub state delegates to HubShiftEngine (single source of truth for override, FMS, confidence).
 *
 * <p>Each flickery condition (shooter, indexer, vision, confidence) gets its own falling-edge
 * debounce so a brief dip on one doesn't reset the hold window for another. hasBall and
 * fireAuthorized respond instantly because you never want to fire without a ball or authorization.
 */
public class ScoringReadiness {
    private static ScoringReadiness instance;

    private static final TunableNumber headingToleranceDeg =
            new TunableNumber(
                    "Scoring/HeadingToleranceDeg", SpatialLaunchConstants.HEADING_TOLERANCE_DEG);

    // once heading reaches tolerance, require 4x the error before declaring off-target.
    // prevents the "almost ready, not ready, almost ready" flicker during SOTM when
    // heading oscillates near the tolerance edge.
    private static final double HEADING_HYSTERESIS_FACTOR = 4.0;

    // inputs (set each cycle via setInputs())
    private boolean shooterReady = false;
    private boolean indexerClear = true;
    private boolean visionLocked = false;
    private boolean hasBall = true;
    private double shotConfidence = 0;
    private double headingErrorDeg = 0;
    private boolean headingOnTarget = false;
    private boolean stickyHeadingOnTarget = false;
    private boolean inExclusionZone = false;

    // per-component falling-edge debouncers
    private final FallingDebounce shooterDebounce =
            new FallingDebounce(DeviceHealthConstants.SHOOTER_READY_DEBOUNCE_SEC);
    private final FallingDebounce indexerDebounce =
            new FallingDebounce(DeviceHealthConstants.INDEXER_CLEAR_DEBOUNCE_SEC);
    private final FallingDebounce visionDebounce =
            new FallingDebounce(DeviceHealthConstants.VISION_LOCKED_DEBOUNCE_SEC);
    private final FallingDebounce confidenceDebounce =
            new FallingDebounce(DeviceHealthConstants.SHOT_CONFIDENT_DEBOUNCE_SEC);

    private boolean debouncedShooterReady = false;
    private boolean debouncedIndexerClear = false;
    private boolean debouncedVisionLocked = false;
    private boolean debouncedShotConfident = false;

    private boolean hubActive = true;
    private boolean readyToShoot = false;
    private int hubShiftNumber = 0;
    private double timeToNextShiftSec = 0;
    private boolean wonAuto = false;
    private boolean wonAutoFromFMS = false;

    private boolean previousReadyToShoot = false;
    private boolean readyStateChanged = false;
    private String readyLostReason = "";

    private double readySinceTimestamp = 0;
    private double timeSinceReadyMs = 0;
    private boolean wasReady = false;

    public static ScoringReadiness getInstance() {
        if (instance == null) {
            instance = new ScoringReadiness();
        }
        return instance;
    }

    private ScoringReadiness() {}

    /**
     * Feed the raw input signals each cycle. Called from ScoringTelemetry.update(), immediately
     * followed by update() in the same method so the composite is fresh for logging.
     */
    public void setInputs(
            boolean shooterReady,
            boolean indexerClear,
            boolean visionLocked,
            boolean hasBall,
            double shotConfidence) {
        this.shooterReady = shooterReady;
        this.indexerClear = indexerClear;
        this.visionLocked = visionLocked;
        this.hasBall = hasBall;
        this.shotConfidence = shotConfidence;
    }

    /**
     * Feed heading error for the heading-on-target gate. Call after setInputs().
     *
     * @param headingErrorDeg absolute heading error in degrees
     * @param inExclusionZone true if robot is in a trench exclusion zone
     */
    public void setHeadingAndZone(double headingErrorDeg, boolean inExclusionZone) {
        this.headingErrorDeg = headingErrorDeg;
        this.inExclusionZone = inExclusionZone;

        double tol = headingToleranceDeg.get();
        double absError = Math.abs(headingErrorDeg);

        // hysteretic heading gate: tight band to enter "on target", wider band to exit
        if (!stickyHeadingOnTarget) {
            headingOnTarget = absError < tol;
            if (headingOnTarget) {
                stickyHeadingOnTarget = true;
            }
        } else {
            if (absError >= tol * HEADING_HYSTERESIS_FACTOR) {
                stickyHeadingOnTarget = false;
                headingOnTarget = false;
            } else {
                headingOnTarget = true;
            }
        }
    }

    /** Run the ReadyToShoot computation. Call once per robotPeriodic(). */
    public void update() {
        double now = Timer.getFPGATimestamp();

        previousReadyToShoot = readyToShoot;

        // Hub state comes from HubShiftEngine (single source of truth for override, FMS,
        // confidence)
        try {
            HubShiftEngine engine = HubShiftEngine.getInstance();
            HubShiftEngine.ShiftInfo info = engine.getOfficialInfo();
            hubActive = info.hubActive();
            wonAuto = engine.isWonAuto();
            wonAutoFromFMS = engine.isWonAutoFromFMS();

            // Map engine phase to shift number for per-shift stats.
            // -1 for non-shift phases so MatchStatsTelemetry doesn't miscount them as endgame.
            hubShiftNumber =
                    switch (info.phase()) {
                        case SHIFT1 -> 1;
                        case SHIFT2 -> 2;
                        case SHIFT3 -> 3;
                        case SHIFT4 -> 4;
                        case ENDGAME -> 0;
                        default -> -1;
                    };

            if (!hubActive) {
                timeToNextShiftSec = info.timeToNextActive();
            } else {
                timeToNextShiftSec = info.timeUntilDeactivation();
            }
        } catch (Throwable t) {
            // HubShiftEngine not ready, safe defaults
            hubActive = true;
            hubShiftNumber = 0;
            timeToNextShiftSec = 0;
        }

        debouncedShooterReady = shooterDebounce.calculate(shooterReady);
        debouncedIndexerClear = indexerDebounce.calculate(indexerClear);
        debouncedVisionLocked = visionDebounce.calculate(visionLocked);
        debouncedShotConfident = confidenceDebounce.calculate(shotConfidence >= 50);

        // 8-condition composite. fireAuthorized covers hubActive with TOF-compensated timing.
        // headingOnTarget prevents firing while still rotating to the target.
        // !inExclusionZone prevents firing from under trench ceilings.
        readyToShoot =
                debouncedShooterReady
                        && debouncedIndexerClear
                        && debouncedVisionLocked
                        && hasBall
                        && isFireAuthorized()
                        && debouncedShotConfident
                        && headingOnTarget
                        && !inExclusionZone;

        readyStateChanged = (readyToShoot != previousReadyToShoot);
        if (readyStateChanged && !readyToShoot) {
            readyLostReason = buildNotReadyReason();
        } else if (readyStateChanged && readyToShoot) {
            readyLostReason = "";
        }

        if (readyToShoot && !wasReady) {
            readySinceTimestamp = now;
        }
        timeSinceReadyMs = readyToShoot ? (now - readySinceTimestamp) * 1000.0 : 0;
        wasReady = readyToShoot;
    }

    private String buildNotReadyReason() {
        StringBuilder sb = new StringBuilder();
        if (!debouncedShooterReady) sb.append("Shooter");
        if (!debouncedIndexerClear) {
            if (sb.length() > 0) sb.append("+");
            sb.append("Indexer");
        }
        if (!debouncedVisionLocked) {
            if (sb.length() > 0) sb.append("+");
            sb.append("Vision");
        }
        if (!isFireAuthorized()) {
            if (sb.length() > 0) sb.append("+");
            sb.append("FireAuth");
        }
        if (!hasBall) {
            if (sb.length() > 0) sb.append("+");
            sb.append("NoBall");
        }
        if (!debouncedShotConfident) {
            if (sb.length() > 0) sb.append("+");
            sb.append("Confidence");
        }
        if (!headingOnTarget) {
            if (sb.length() > 0) sb.append("+");
            sb.append("Heading");
        }
        if (inExclusionZone) {
            if (sb.length() > 0) sb.append("+");
            sb.append("ExclusionZone");
        }
        return sb.length() > 0 ? sb.toString() : "Unknown";
    }

    public boolean isReadyToShoot() {
        return readyToShoot;
    }

    public boolean isHubActive() {
        return hubActive;
    }

    public int getHubShiftNumber() {
        return hubShiftNumber;
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

    public double getTimeSinceReadyMs() {
        return timeSinceReadyMs;
    }

    public boolean isWonAuto() {
        return wonAuto;
    }

    public boolean isWonAutoFromFMS() {
        return wonAutoFromFMS;
    }

    /** Reset all state. Use for testing and match transitions (teleopInit). */
    public void reset() {
        shooterReady = false;
        indexerClear = true;
        visionLocked = false;
        hasBall = true;
        shotConfidence = 0;
        headingErrorDeg = 0;
        headingOnTarget = false;
        stickyHeadingOnTarget = false;
        inExclusionZone = false;
        hubActive = true;
        readyToShoot = false;
        hubShiftNumber = 0;
        timeToNextShiftSec = 0;
        wonAuto = false;
        wonAutoFromFMS = false;
        shooterDebounce.reset();
        indexerDebounce.reset();
        visionDebounce.reset();
        confidenceDebounce.reset();
        debouncedShooterReady = false;
        debouncedIndexerClear = false;
        debouncedVisionLocked = false;
        debouncedShotConfident = false;
        previousReadyToShoot = false;
        readyStateChanged = false;
        readyLostReason = "";
        readySinceTimestamp = 0;
        timeSinceReadyMs = 0;
        wasReady = false;
    }

    public boolean isShooterReady() {
        return shooterReady;
    }

    public boolean isIndexerClear() {
        return indexerClear;
    }

    public boolean isVisionLocked() {
        return visionLocked;
    }

    public boolean isHasBall() {
        return hasBall;
    }

    public boolean isShotConfident() {
        return shotConfidence >= 50;
    }

    public double getShotConfidence() {
        return shotConfidence;
    }

    public boolean isHeadingOnTarget() {
        return headingOnTarget;
    }

    public double getHeadingErrorDeg() {
        return headingErrorDeg;
    }

    public boolean isInExclusionZone() {
        return inExclusionZone;
    }

    public boolean isDebouncedShooterReady() {
        return debouncedShooterReady;
    }

    public boolean isDebouncedIndexerClear() {
        return debouncedIndexerClear;
    }

    public boolean isDebouncedVisionLocked() {
        return debouncedVisionLocked;
    }

    public boolean isDebouncedShotConfident() {
        return debouncedShotConfident;
    }

    /** Delegates to FireAuthorization. Fail-open: permits firing if FireAuthorization throws. */
    public boolean isFireAuthorized() {
        try {
            return FireAuthorization.getInstance().isAuthorized();
        } catch (Throwable t) {
            return true;
        }
    }

    public String getFireAuthLevel() {
        try {
            return FireAuthorization.getInstance().getLevel().name();
        } catch (Throwable t) {
            return "FIRE_AUTHORIZED";
        }
    }

    /** Time remaining in the ballistic window (shifted by TOF). Positive = window open. */
    public double getBallisticWindowRemainingSec() {
        try {
            HubShiftEngine engine = HubShiftEngine.getInstance();
            if (engine.isFeatureEnabled()) {
                HubShiftEngine.ShiftInfo shifted = engine.getShiftedInfo();
                if (shifted.hubActive()) {
                    return shifted.timeUntilDeactivation();
                } else {
                    return -shifted.timeToNextActive();
                }
            }
        } catch (Throwable t) {
            // fall through
        }
        return 0;
    }

    /**
     * Falling-edge hold: when the input drops from true to false, the output stays true for holdSec
     * before following. Rising edges pass through immediately. Won't falsely report true at startup
     * before the input has ever been true (unlike WPILib's Debouncer with kFalling baseline=true).
     */
    static class FallingDebounce {
        private final double holdSec;
        private boolean output = false;
        private double falseSince = 0;

        FallingDebounce(double holdSec) {
            this.holdSec = holdSec;
        }

        boolean calculate(boolean input) {
            double now = Timer.getFPGATimestamp();
            if (input) {
                output = true;
                falseSince = 0;
                return true;
            }
            if (!output) {
                return false;
            }
            if (falseSince == 0) {
                falseSince = now;
            }
            if ((now - falseSince) >= holdSec) {
                output = false;
            }
            return output;
        }

        void reset() {
            output = false;
            falseSince = 0;
        }
    }
}
