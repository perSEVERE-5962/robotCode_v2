package frc.robot.telemetry;

import frc.robot.util.ScoringReadiness;
import frc.robot.util.SpatialLaunchValidator;

/**
 * Scoring readiness telemetry: feeds raw inputs into ScoringReadiness, triggers the composite
 * computation, then logs all values. The update-then-log sequence makes sure logged ReadyToShoot
 * reflects the current cycle's inputs, not the previous cycle's.
 */
public class ScoringTelemetry implements SubsystemTelemetry {
    private final ShooterTelemetry shooterTelemetry;
    private final IndexerTelemetry indexerTelemetry;
    private final VisionTelemetry visionTelemetry;

    private boolean scoringAvailable = false;

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
            if (shooterTelemetry == null || indexerTelemetry == null || visionTelemetry == null) {
                scoringAvailable = false;
                ScoringReadiness.getInstance().reset();
                return;
            }
            scoringAvailable = true;

            ScoringReadiness sr = ScoringReadiness.getInstance();

            // pass-through until ShotCalculator is wired on the SOTM branch
            double confidence = 100;

            sr.setInputs(
                    shooterTelemetry.isAtSpeed(),
                    !indexerTelemetry.isJamDetected(),
                    visionTelemetry.isLockedOnTarget(),
                    true, // stub: assume ball present until hopper sensor is wired
                    confidence);

            // Stub: heading error = 0 (pass-through) until ShotCalculator provides aim direction
            double headingError = 0;
            boolean inExclusionZone = false;
            try {
                var pose = frc.robot.RobotContainer.getInstance().getSwerveSubsystem().getPose();
                inExclusionZone =
                        SpatialLaunchValidator.isInExclusionZone(pose.getX(), pose.getY());
            } catch (Throwable t) {
                // pose not available yet, fail-open
            }
            sr.setHeadingAndZone(headingError, inExclusionZone);

            sr.update();
        } catch (Throwable t) {
            scoringAvailable = false;
            ScoringReadiness.getInstance().reset();
        }
    }

    @Override
    public void log() {
        ScoringReadiness sr = ScoringReadiness.getInstance();

        SafeLog.put("Scoring/Available", scoringAvailable);
        SafeLog.put("Scoring/ReadyToShoot", sr.isReadyToShoot());
        SafeLog.put("Scoring/Conditions/ShooterReady", sr.isShooterReady());
        SafeLog.put("Scoring/Conditions/IndexerClear", sr.isIndexerClear());
        SafeLog.put("Scoring/Conditions/VisionLocked", sr.isVisionLocked());
        SafeLog.put("Scoring/Conditions/HasBall", sr.isHasBall());
        SafeLog.put("Scoring/Conditions/HubActive", sr.isHubActive());
        SafeLog.put("Scoring/Conditions/FireAuthorized", sr.isFireAuthorized());
        SafeLog.put("Scoring/Conditions/ShotConfident", sr.isShotConfident());
        SafeLog.put("Scoring/HeadingOnTarget", sr.isHeadingOnTarget());
        SafeLog.put("Scoring/HeadingErrorDeg", sr.getHeadingErrorDeg());
        SafeLog.put("Scoring/InExclusionZone", sr.isInExclusionZone());

        // debounced values show what the composite actually uses after per-component smoothing
        SafeLog.put("Scoring/Debounced/ShooterReady", sr.isDebouncedShooterReady());
        SafeLog.put("Scoring/Debounced/IndexerClear", sr.isDebouncedIndexerClear());
        SafeLog.put("Scoring/Debounced/VisionLocked", sr.isDebouncedVisionLocked());
        SafeLog.put("Scoring/Debounced/ShotConfident", sr.isDebouncedShotConfident());

        SafeLog.put("Scoring/BallisticWindowRemainingSec", sr.getBallisticWindowRemainingSec());
        SafeLog.put("Scoring/TimeSinceReadyMs", sr.getTimeSinceReadyMs());
        SafeLog.put("Scoring/HubShiftNumber", sr.getHubShiftNumber());
        SafeLog.put("Scoring/TimeToNextShiftSec", sr.getTimeToNextShiftSec());
        SafeLog.put("Scoring/Conditions/WonAuto", sr.isWonAuto());
        SafeLog.put("Scoring/Conditions/WonAutoFromFMS", sr.isWonAutoFromFMS());

        SafeLog.put("Scoring/ReadyStateChanged", sr.isReadyStateChanged());
        SafeLog.put("Scoring/ReadyLostReason", sr.getReadyLostReason());
    }

    @Override
    public String getName() {
        return "Scoring";
    }

    public boolean isReadyToShoot() {
        return ScoringReadiness.getInstance().isReadyToShoot();
    }

    public int getHubShiftNumber() {
        return ScoringReadiness.getInstance().getHubShiftNumber();
    }

    public boolean isHubActive() {
        return ScoringReadiness.getInstance().isHubActive();
    }

    public boolean isFireAuthorized() {
        return ScoringReadiness.getInstance().isFireAuthorized();
    }

    public double getTimeToNextShiftSec() {
        return ScoringReadiness.getInstance().getTimeToNextShiftSec();
    }

    public double getBallisticWindowRemainingSec() {
        return ScoringReadiness.getInstance().getBallisticWindowRemainingSec();
    }

    public boolean isReadyStateChanged() {
        return ScoringReadiness.getInstance().isReadyStateChanged();
    }

    public String getReadyLostReason() {
        return ScoringReadiness.getInstance().getReadyLostReason();
    }

    public String getFireAuthLevel() {
        return ScoringReadiness.getInstance().getFireAuthLevel();
    }

    public boolean isWonAuto() {
        return ScoringReadiness.getInstance().isWonAuto();
    }
}
