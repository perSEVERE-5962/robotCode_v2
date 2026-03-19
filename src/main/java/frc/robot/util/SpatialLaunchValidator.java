package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants.HubScoringConstants;
import frc.robot.Constants.SpatialLaunchConstants;
import frc.robot.telemetry.SafeLog;

/**
 * Field position validation for launching. Three gate categories: path obstruction (pillars),
 * minimum distance, and trench ceiling exclusion zones. Velocity-predictive trench avoidance
 * expands exclusion zones proportional to closing speed.
 *
 * <p>Pure geometry, no mechanism dependency. Works for any launcher angle.
 */
public final class SpatialLaunchValidator {

    private static final TunableNumber minLaunchDistance =
            new TunableNumber(
                    "SpatialLaunch/MinDistanceM", SpatialLaunchConstants.MIN_LAUNCH_DISTANCE_M);
    private static final TunableNumber retractionTimeSec =
            new TunableNumber(
                    "SpatialLaunch/RetractionTimeSec", SpatialLaunchConstants.RETRACTION_TIME_SEC);

    public static final class ValidationResult {
        public final boolean pathClear;
        public final boolean distanceValid;
        public final boolean outsideExclusionZone;
        public final double distanceToHub;
        public final double nearestTrenchDistM;

        ValidationResult(
                boolean pathClear,
                boolean distanceValid,
                boolean outsideExclusionZone,
                double distanceToHub,
                double nearestTrenchDistM) {
            this.pathClear = pathClear;
            this.distanceValid = distanceValid;
            this.outsideExclusionZone = outsideExclusionZone;
            this.distanceToHub = distanceToHub;
            this.nearestTrenchDistM = nearestTrenchDistM;
        }

        public boolean isValid() {
            return pathClear && distanceValid && outsideExclusionZone;
        }
    }

    public static ValidationResult validate(Pose2d robotPose) {
        Translation2d hub = getHubCenter();
        Translation2d robot = robotPose.getTranslation();
        double dist = robot.getDistance(hub);

        boolean pathClear = isPathClear(robot, hub);
        boolean distanceValid = dist >= minLaunchDistance.get();
        boolean outsideExclusion = !isInExclusionZone(robot.getX(), robot.getY());
        double nearestTrench = getNearestTrenchDistM(robot.getX(), robot.getY());

        return new ValidationResult(
                pathClear, distanceValid, outsideExclusion, dist, nearestTrench);
    }

    /** True if the robot is inside any trench ceiling zone. Shooting from here hits the ceiling. */
    public static boolean isInExclusionZone(double robotX, double robotY) {
        for (double[] zone : SpatialLaunchConstants.TRENCH_EXCLUSION_ZONES) {
            if (robotX >= zone[0] && robotX <= zone[2] && robotY >= zone[1] && robotY <= zone[3]) {
                return true;
            }
        }
        return false;
    }

    /** Distance to the nearest trench zone edge. Returns 0 if inside a zone. */
    public static double getNearestTrenchDistM(double robotX, double robotY) {
        double minDist = Double.MAX_VALUE;
        for (double[] zone : SpatialLaunchConstants.TRENCH_EXCLUSION_ZONES) {
            double dx = Math.max(0, Math.max(zone[0] - robotX, robotX - zone[2]));
            double dy = Math.max(0, Math.max(zone[1] - robotY, robotY - zone[3]));
            double dist = Math.sqrt(dx * dx + dy * dy);
            minDist = Math.min(minDist, dist);
        }
        return minDist;
    }

    /**
     * Expands the exclusion zone by closingSpeed * retractionTime so the driver gets warned before
     * entering. Positive closing speed means approaching.
     */
    public static double computeTrenchBuffer(double closingSpeedMps) {
        if (closingSpeedMps <= 0) {
            return 0;
        }
        return closingSpeedMps * retractionTimeSec.get();
    }

    /** True if robot is in the velocity-expanded exclusion zone (should warn driver). */
    public static boolean isInExpandedExclusionZone(
            double robotX, double robotY, double velocityX, double velocityY) {
        if (isInExclusionZone(robotX, robotY)) {
            return true;
        }

        double speed = Math.sqrt(velocityX * velocityX + velocityY * velocityY);
        if (speed < 0.1) {
            return false;
        }

        double buffer = computeTrenchBuffer(speed);
        if (buffer <= 0) {
            return false;
        }

        double nearestDist = getNearestTrenchDistM(robotX, robotY);
        return nearestDist < buffer;
    }

    public static void logTelemetry(
            ValidationResult result,
            double closingSpeedMps,
            double bufferM,
            boolean exclusionWarning) {
        SafeLog.put("Scoring/InExclusionZone", !result.outsideExclusionZone);
        SafeLog.put("Scoring/NearestTrenchDistM", result.nearestTrenchDistM);
        SafeLog.put("Scoring/TrenchClosingSpeedMps", closingSpeedMps);
        SafeLog.put("Scoring/TrenchBufferM", bufferM);
        SafeLog.put("Scoring/ExclusionZoneWarning", exclusionWarning);
    }

    /** 2D ray-cast from robot to hub. Returns true if no trench pillar blocks the path. */
    static boolean isPathClear(Translation2d robot, Translation2d hub) {
        double margin = SpatialLaunchConstants.BALL_RADIUS_M;

        for (double[] pillar : SpatialLaunchConstants.TRENCH_PILLARS) {
            if (segmentIntersectsAABB(
                    robot.getX(),
                    robot.getY(),
                    hub.getX(),
                    hub.getY(),
                    pillar[0] - margin,
                    pillar[1] - margin,
                    pillar[2] + margin,
                    pillar[3] + margin)) {
                return false;
            }
        }
        return true;
    }

    /** 2D line segment / axis-aligned bounding box intersection via slab method. */
    static boolean segmentIntersectsAABB(
            double x0,
            double y0,
            double x1,
            double y1,
            double minX,
            double minY,
            double maxX,
            double maxY) {

        double dx = x1 - x0;
        double dy = y1 - y0;

        double tMin = 0.0;
        double tMax = 1.0;

        if (Math.abs(dx) < 1e-12) {
            if (x0 < minX || x0 > maxX) {
                return false;
            }
        } else {
            double invDx = 1.0 / dx;
            double t1 = (minX - x0) * invDx;
            double t2 = (maxX - x0) * invDx;
            if (t1 > t2) {
                double tmp = t1;
                t1 = t2;
                t2 = tmp;
            }
            tMin = Math.max(tMin, t1);
            tMax = Math.min(tMax, t2);
            if (tMin > tMax) {
                return false;
            }
        }

        if (Math.abs(dy) < 1e-12) {
            if (y0 < minY || y0 > maxY) {
                return false;
            }
        } else {
            double invDy = 1.0 / dy;
            double t1 = (minY - y0) * invDy;
            double t2 = (maxY - y0) * invDy;
            if (t1 > t2) {
                double tmp = t1;
                t1 = t2;
                t2 = tmp;
            }
            tMin = Math.max(tMin, t1);
            tMax = Math.min(tMax, t2);
            if (tMin > tMax) {
                return false;
            }
        }

        return true;
    }

    private static Translation2d getHubCenter() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return HubScoringConstants.RED_HUB_CENTER;
        }
        return HubScoringConstants.BLUE_HUB_CENTER;
    }

    private SpatialLaunchValidator() {}
}
