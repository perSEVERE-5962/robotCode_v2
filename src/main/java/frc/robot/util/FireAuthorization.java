package frc.robot.util;

import frc.robot.Constants.HubTimingConstants;
import frc.robot.telemetry.SafeLog;
import frc.robot.util.HubShiftEngine.ScheduleConfidence;
import frc.robot.util.HubShiftEngine.ShiftInfo;

/**
 * 5-level fire authorization based on whether a shot would land while the hub is counting.
 * Compares projectile landing time against the hub's scoring window each cycle.
 */
public class FireAuthorization {
    private static FireAuthorization instance;

    public enum AuthorizationLevel {
        FIRE_AUTHORIZED,
        FIRE_MARGINAL,
        PRE_SPIN,
        HOLD_TIMING,
        HOLD_INACTIVE
    }

    private final TunableNumber safeMargin =
            new TunableNumber("FireAuth/SafeMarginSec", HubTimingConstants.FIRE_SAFE_MARGIN_SEC);
    private final TunableNumber preSpinWindow =
            new TunableNumber("FireAuth/PreSpinWindowSec", HubTimingConstants.PRE_SPIN_WINDOW_SEC);
    private final TunableNumber fallbackMarginExtra =
            new TunableNumber(
                    "FireAuth/FallbackExtraMarginSec",
                    HubTimingConstants.FALLBACK_MARGIN_EXTRA_SEC);

    // fail-open default: a robot that won't fire is worse than one that fires at a bad time
    private AuthorizationLevel level = AuthorizationLevel.FIRE_AUTHORIZED;
    private double margin = 0;
    private double ballLandingTime = 0;
    private double windowRemaining = 0;
    private boolean authorized = true;
    private boolean spatiallyBlocked = false;

    private FireAuthorization() {}

    public static FireAuthorization getInstance() {
        if (instance == null) {
            instance = new FireAuthorization();
        }
        return instance;
    }

    /** Call every robotPeriodic() after HubShiftEngine.update(). */
    public void update(
            ShiftInfo shiftedInfo,
            double currentTOF,
            ScheduleConfidence confidence,
            boolean hubTrackingEnabled) {

        if (!hubTrackingEnabled) {
            level = AuthorizationLevel.FIRE_AUTHORIZED;
            margin = Double.MAX_VALUE;
            ballLandingTime = 0;
            windowRemaining = Double.MAX_VALUE;
            authorized = true;
            logTelemetry();
            return;
        }

        // Spatial exclusion (trench ceilings) is handled by the !inExclusionZone gate
        // in ScoringReadiness. FireAuthorization focuses on timing only.
        spatiallyBlocked = false;

        HubShiftEngine engine = HubShiftEngine.getInstance();
        double fuelDelay = (engine.getFuelCountDelayMin() + engine.getFuelCountDelayMax()) / 2.0;
        double extension = engine.getFuelCountExtension();

        ballLandingTime = currentTOF + fuelDelay;

        double effectiveSafeMargin = safeMargin.get();
        if (confidence == ScheduleConfidence.FALLBACK) {
            effectiveSafeMargin += fallbackMarginExtra.get();
        }

        if (shiftedInfo.hubActive()) {
            windowRemaining = shiftedInfo.timeUntilDeactivation();
            margin = windowRemaining - ballLandingTime;

            if (margin >= effectiveSafeMargin) {
                level = AuthorizationLevel.FIRE_AUTHORIZED;
            } else if (margin > -extension) {
                level = AuthorizationLevel.FIRE_MARGINAL;
            } else {
                level = AuthorizationLevel.HOLD_TIMING;
            }
        } else {
            windowRemaining = 0;
            double timeToActive = shiftedInfo.timeToNextActive();
            margin = -(timeToActive);

            if (timeToActive > 0 && timeToActive <= preSpinWindow.get()) {
                level = AuthorizationLevel.PRE_SPIN;
            } else {
                level = AuthorizationLevel.HOLD_INACTIVE;
            }
        }

        authorized =
                (level == AuthorizationLevel.FIRE_AUTHORIZED
                        || level == AuthorizationLevel.FIRE_MARGINAL);

        logTelemetry();
    }

    private void logTelemetry() {
        SafeLog.put("Scoring/FireAuth/Level", level.name());
        SafeLog.put("Scoring/FireAuth/Margin", margin);
        SafeLog.put("Scoring/FireAuth/BallLandingTime", ballLandingTime);
        SafeLog.put("Scoring/FireAuth/WindowRemaining", windowRemaining);
        SafeLog.put("Scoring/FireAuth/Authorized", authorized);
        SafeLog.put("Scoring/FireAuth/SpatiallyBlocked", spatiallyBlocked);
    }

    public AuthorizationLevel getLevel() {
        return level;
    }

    public double getMargin() {
        return margin;
    }

    public double getBallLandingTime() {
        return ballLandingTime;
    }

    public double getWindowRemaining() {
        return windowRemaining;
    }

    public boolean isAuthorized() {
        return authorized;
    }

    public boolean isPreSpin() {
        return level == AuthorizationLevel.PRE_SPIN;
    }

    static void resetInstance() {
        instance = null;
    }
}
