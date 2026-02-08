package frc.robot.telemetry;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DeviceHealthConstants;

/** Scoring readiness: composite ReadyToShoot from shooter + indexer + vision. */
public class ScoringTelemetry implements SubsystemTelemetry {
    private final ShooterTelemetry shooterTelemetry;
    private final IndexerTelemetry indexerTelemetry;
    private final VisionTelemetry visionTelemetry;

    // Hub shift boundaries (teleop matchTime counts down from ~135)
    // Game Manual Section 6.4: 5s transition, 4x25s shifts, 30s endgame
    private static final double SHIFT_1_START = 130.0;
    private static final double SHIFT_2_START = 105.0;
    private static final double SHIFT_3_START = 80.0;
    private static final double SHIFT_4_START = 55.0;
    private static final double ENDGAME_START = 30.0;

    // ReadyToShoot stays true through brief velocity dips during sustained fire.
    // Manual time-based debounce (not WPILib Debouncer — that has wrong initial baseline).
    private double readyFalseSince = 0;

    private boolean scoringAvailable = false;
    private boolean shooterReady = false;
    private boolean indexerClear = false;
    private boolean visionLocked = false;
    private boolean hasBall = true;      // Stub: assume ball present until hopper sensor
    private boolean hubActive = true;
    private boolean readyToShoot = false;

    // Hub timing state
    private boolean wonAuto = false;
    private int hubShiftNumber = 0;
    private double timeToNextShiftSec = 0;

    // Time tracking
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

            shooterReady = shooterTelemetry.isAtSpeed();
            indexerClear = !indexerTelemetry.isJamDetected();
            visionLocked = visionTelemetry.isLockedOnTarget();

            hasBall = true;    // Stub: read from hopper sensor when available

            // Hub active/inactive from match timer (REBUILT Section 6.4)
            wonAuto = SmartDashboard.getBoolean("WonAuto", false);
            if (DriverStation.isTeleop()) {
                double matchTime = DriverStation.getMatchTime();
                if (matchTime < 0) {
                    hubActive = true; // practice mode, no FMS -- fail-safe to active
                } else {
                    hubShiftNumber = computeShiftNumber(matchTime);
                    hubActive = computeHubActive(matchTime, wonAuto);
                    timeToNextShiftSec = computeTimeToNextShift(matchTime);
                }
            } else {
                hubActive = true; // auto, disabled, test: all hubs active
                hubShiftNumber = 0;
                timeToNextShiftSec = 0;
            }

            boolean rawReady = shooterReady && indexerClear && visionLocked && hasBall && hubActive;
            if (rawReady) {
                readyToShoot = true;
                readyFalseSince = 0;
            } else if (!readyToShoot) {
                readyFalseSince = 0;
            } else {
                // Was true, conditions now false — hold true during debounce window
                if (readyFalseSince == 0) readyFalseSince = now;
                if ((now - readyFalseSince) >= DeviceHealthConstants.READY_TO_SHOOT_DEBOUNCE_SEC) {
                    readyToShoot = false;
                }
            }

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
        indexerClear = true;  // Safe default: assume clear
        visionLocked = false;
        readyToShoot = false;
        timeSinceReadyMs = 0;
        hubShiftNumber = 0;
        timeToNextShiftSec = 0;
    }

    /** Odd shifts (1,3): winner INACTIVE. Even shifts (2,4): winner ACTIVE. */
    private boolean computeHubActive(double matchTime, boolean wonAuto) {
        if (matchTime > SHIFT_1_START || matchTime <= ENDGAME_START) {
            return true; // transition or endgame: both alliances active
        }
        int shift = computeShiftNumber(matchTime);
        boolean oddShift = (shift % 2 == 1);
        return wonAuto ? !oddShift : oddShift;
    }

    private int computeShiftNumber(double matchTime) {
        if (matchTime > SHIFT_1_START) return 0;
        if (matchTime > SHIFT_2_START) return 1;
        if (matchTime > SHIFT_3_START) return 2;
        if (matchTime > SHIFT_4_START) return 3;
        if (matchTime > ENDGAME_START) return 4;
        return 0; // endgame
    }

    private double computeTimeToNextShift(double matchTime) {
        if (matchTime > SHIFT_1_START) return matchTime - SHIFT_1_START;
        if (matchTime > SHIFT_2_START) return matchTime - SHIFT_2_START;
        if (matchTime > SHIFT_3_START) return matchTime - SHIFT_3_START;
        if (matchTime > SHIFT_4_START) return matchTime - SHIFT_4_START;
        if (matchTime > ENDGAME_START) return matchTime - ENDGAME_START;
        return matchTime; // time remaining in match
    }

    @Override
    public void log() {
        SafeLog.put("Scoring/Available", scoringAvailable);
        SafeLog.put("Scoring/ReadyToShoot", readyToShoot);
        SafeLog.put("Scoring/Conditions/ShooterReady", shooterReady);
        SafeLog.put("Scoring/Conditions/IndexerClear", indexerClear);
        SafeLog.put("Scoring/Conditions/VisionLocked", visionLocked);
        SafeLog.put("Scoring/Conditions/HasBall", hasBall);
        SafeLog.put("Scoring/Conditions/HubActive", hubActive);
        SafeLog.put("Scoring/TimeSinceReadyMs", timeSinceReadyMs);
        SafeLog.put("Scoring/HubShiftNumber", hubShiftNumber);
        SafeLog.put("Scoring/TimeToNextShiftSec", timeToNextShiftSec);
        SafeLog.put("Scoring/Conditions/WonAuto", wonAuto);
    }

    @Override
    public String getName() {
        return "Scoring";
    }

    // Accessors for TelemetryManager
    public boolean isReadyToShoot() { return readyToShoot; }
}
