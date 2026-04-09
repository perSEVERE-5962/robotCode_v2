package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Single source of truth for all 2026 REBUILT field geometry. Coordinates use WPILib convention:
 * origin at blue alliance wall corner, X+ toward red wall, Y+ toward far side wall. Units: meters.
 *
 * <p>Values from game manual Section 5, cross-referenced with YAGSL Arena2026Rebuilt collision
 * geometry and BallPhysicsSim verified measurements. Where sources disagree, game manual wins.
 */
public final class FieldGeometry {

  private FieldGeometry() {}

  // Field Envelope

  public static final double FIELD_LENGTH_M = 16.541; // 651.22in
  public static final double FIELD_WIDTH_M = 8.070; // 317.7in (game manual)
  public static final Translation2d FIELD_CENTER =
      new Translation2d(FIELD_LENGTH_M / 2.0, FIELD_WIDTH_M / 2.0);
  public static final double CENTER_LINE_X = FIELD_LENGTH_M / 2.0; // 8.2705m

  // wall heights (for ballistic trajectory checks)
  public static final double ALLIANCE_WALL_HEIGHT_M = 0.935; // 36.8in
  public static final double GUARDRAIL_HEIGHT_M = 0.508; // 20in polycarbonate

  // Zone Boundaries

  // alliance zone: from wall to starting line (158.6in deep)
  public static final double ALLIANCE_ZONE_DEPTH_M = 4.028; // 158.6in
  public static final double BLUE_ALLIANCE_LINE_X = ALLIANCE_ZONE_DEPTH_M; // 4.028
  public static final double RED_ALLIANCE_LINE_X = FIELD_LENGTH_M - ALLIANCE_ZONE_DEPTH_M; // 12.513

  // neutral zone: 283in deep between the two alliance zones
  public static final double NEUTRAL_ZONE_DEPTH_M = 7.189; // 283in

  // Ball Specs

  public static final double FUEL_DIAMETER_M = 0.1501; // 5.91in
  public static final double FUEL_MASS_KG = 0.215; // 7.6oz

  // Scoring Hubs

  public static final class Hub {
    // hub centers from YAGSL Arena2026Rebuilt (matches MapleSim physics).
    // AprilTag-derived values differ by ~28mm in X. verify on real field if needed.
    public static final Translation2d BLUE_CENTER = new Translation2d(4.5974, 4.035);
    public static final Translation2d RED_CENTER = new Translation2d(11.938, 4.035);

    // open scoring side: blue shoots from -X, red shoots from +X
    public static final Rotation2d BLUE_SCORING_SIDE = Rotation2d.fromDegrees(180);
    public static final Rotation2d RED_SCORING_SIDE = Rotation2d.fromDegrees(0);

    // hub structure dimensions
    public static final double ENTRY_HEIGHT_M = 1.829; // 72.0in (full structure height)
    public static final double INNER_HEIGHT_M = 1.435; // 56.5in (scoring opening)
    public static final double INNER_WIDTH_M = 1.059; // 41.7in (hexagonal opening apothem x2)
    public static final double ENTRY_APOTHEM_M = 0.5295; // 20.85in across flats / 2
    public static final double BASE_SIDE_M = 1.194; // 47in
    public static final double BASE_HEIGHT_M = 0.5;

    // base AABBs (solid collision, robots can't drive through)
    public static final Rect BLUE_BASE = new Rect(4.000, 3.438, 5.194, 4.632);
    public static final Rect RED_BASE = new Rect(11.341, 3.438, 12.535, 4.632);

    // ramp AABBs (solid ground-level collision around each hub, 47in x 217in)
    public static final double RAMP_HEIGHT_M = 0.165; // 6.5in
    public static final Rect BLUE_RAMP = new Rect(4.000, 1.279, 5.194, 6.791);
    public static final Rect RED_RAMP = new Rect(11.341, 1.279, 12.535, 6.791);

    // nets block shots from field-center side. open side faces the alliance wall.
    public static final double NET_OFFSET_M = 0.858; // BASE_SIDE/2 + 0.261
    public static final double NET_WIDTH_M = 1.484; // Y-extent
    public static final double NET_X_BLUE = 5.455; // center + offset toward field center
    public static final double NET_X_RED = 11.080; // center - offset toward field center
    public static final double NET_Z_MIN = 1.5;
    public static final double NET_Z_MAX = 3.057;

    // scored balls exit here
    public static final Translation2d BLUE_EXIT = new Translation2d(5.3, 4.026);
    public static final Translation2d RED_EXIT = new Translation2d(11.241, 4.026);
    public static final double EXIT_Z = 0.89;

    /** Get the hub center for the given alliance (true = blue). */
    public static Translation2d center(boolean isBlue) {
      return isBlue ? BLUE_CENTER : RED_CENTER;
    }

    /** Get the scoring side rotation for the given alliance. */
    public static Rotation2d scoringSide(boolean isBlue) {
      return isBlue ? BLUE_SCORING_SIDE : RED_SCORING_SIDE;
    }
  }

  // Bumps (tent-shaped, 15 deg ramps)

  public static final class Bump {
    public static final double PEAK_HEIGHT_M = 0.1654; // 6.513in (game manual)
    public static final double RAMP_ANGLE_DEG = 15.0;

    // bump peak X positions (tent apex)
    public static final double BLUE_PEAK_X = 4.524;
    public static final double RED_PEAK_X = 12.017;

    // bump footprint AABBs
    public static final Rect BLUE_LOWER = new Rect(3.960, 1.88, 5.088, 3.73);
    public static final Rect BLUE_UPPER = new Rect(3.960, 4.32, 5.088, 6.17);
    public static final Rect RED_LOWER = new Rect(11.453, 1.88, 12.581, 3.73);
    public static final Rect RED_UPPER = new Rect(11.453, 4.32, 12.581, 6.17);

    // safe Y corridors where no bumps exist
    public static final double SAFE_Y_BELOW = 1.88;
    public static final double SAFE_Y_GAP_MIN = 3.73;
    public static final double SAFE_Y_GAP_MAX = 4.32;
    public static final double SAFE_Y_ABOVE = 6.17;

    public static final Rect[] ALL = {BLUE_LOWER, BLUE_UPPER, RED_LOWER, RED_UPPER};
  }

  // Trenches (4 pillars + 4 ceiling zones)

  public static final class Trench {
    public static final double UNDERPASS_HEIGHT_M = 0.565; // 22.25in clearance
    public static final double TOTAL_HEIGHT_M = 1.022; // 40.25in (game manual)
    public static final double OPENING_WIDTH_M = 1.279; // 50.34in (drivable underpass)
    public static final double STRUCTURE_WIDTH_M = 1.668; // 65.65in (full outer frame)
    public static final double SUPPORT_WIDTH_M = 0.194; // 7.66in (each side support column)

    // pillar AABBs (steel columns, will bend your frame)
    public static final Rect BLUE_NEAR_PILLAR = new Rect(3.960, 1.265, 4.265, 1.570);
    public static final Rect BLUE_FAR_PILLAR = new Rect(3.960, 6.177, 4.265, 6.482);
    public static final Rect RED_NEAR_PILLAR = new Rect(12.276, 1.265, 12.581, 1.570);
    public static final Rect RED_FAR_PILLAR = new Rect(12.276, 6.177, 12.581, 6.482);

    // ceiling zones (ball trajectory hits ceiling if launched from under here)
    public static final Rect BLUE_NEAR_CEILING = new Rect(3.960, 1.57, 5.180, 3.73);
    public static final Rect BLUE_FAR_CEILING = new Rect(3.960, 4.322, 5.180, 6.482);
    public static final Rect RED_NEAR_CEILING = new Rect(11.361, 1.57, 12.581, 3.73);
    public static final Rect RED_FAR_CEILING = new Rect(11.361, 4.322, 12.581, 6.482);

    public static final Rect[] PILLARS = {
      BLUE_NEAR_PILLAR, BLUE_FAR_PILLAR, RED_NEAR_PILLAR, RED_FAR_PILLAR
    };

    public static final Rect[] CEILINGS = {
      BLUE_NEAR_CEILING, BLUE_FAR_CEILING, RED_NEAR_CEILING, RED_FAR_CEILING
    };
  }

  // Alliance Towers

  public static final class Tower {
    // blue tower from BallPhysicsSim, red tower derived via rotational symmetry
    public static final Translation2d BLUE_POLE = new Translation2d(1.067, 4.039);
    public static final Translation2d RED_POLE =
        new Translation2d(FIELD_LENGTH_M - 1.067, FIELD_WIDTH_M - 4.039); // (15.474, 4.031)

    // rung heights (1-1/4in Schedule 40 pipe)
    public static final double RUNG_RADIUS_M = 0.0211; // 1.66in OD / 2
    public static final double RUNG_LOW_M = 0.686; // 27in
    public static final double RUNG_MID_M = 1.143; // 45in
    public static final double RUNG_HIGH_M = 1.600; // 63in
    public static final double RUNG_OVERHANG_M = 0.149; // 5.875in past uprights

    public static final double UPRIGHT_SPACING_M = 0.819; // center-to-center
    public static final double UPRIGHT_HEIGHT_M = 1.831; // 72.1in
  }

  // Fuel Depots and Outposts

  public static final class Depot {
    // outposts in opposing corners
    public static final Rect BLUE_OUTPOST = new Rect(0, 0, 1.803, 3.404);
    public static final Rect RED_OUTPOST = new Rect(14.738, 4.648, 16.541, 8.070);

    // corrals adjacent to outposts
    public static final Rect BLUE_CORRAL = new Rect(1.803, 0, 2.712, 0.955);
    public static final Rect RED_CORRAL = new Rect(13.829, 7.115, 14.738, 8.070);

    // outpost chute dimensions (where human player feeds balls)
    public static final double CHUTE_WIDTH_M = 0.808; // 31.8in
    public static final double CHUTE_HEIGHT_FROM_FLOOR_M = 0.714; // 28.1in

    // depot structures along alliance wall
    public static final double DEPOT_WIDTH_M = 1.067; // 42in
    public static final double DEPOT_DEPTH_M = 0.686; // 27in
  }

  // Fuel Pool (neutral zone ball spawn area, centered on field)

  public static final class FuelPool {
    public static final double WIDTH_M = 4.620; // 181.9in
    public static final double HEIGHT_M = 1.826; // 71.9in
    public static final Rect BOUNDS =
        Rect.fromCenter(FIELD_CENTER.getX(), FIELD_CENTER.getY(), WIDTH_M, HEIGHT_M);
  }

  // AprilTag ID Ranges

  public static final class AprilTags {
    public static final int BLUE_MIN = 1;
    public static final int BLUE_MAX = 16;
    public static final int RED_MIN = 17;
    public static final int RED_MAX = 32;
    public static final double TAG_WIDTH_M = 0.165; // 6.5in

    public static boolean isBlueTag(int id) {
      return id >= BLUE_MIN && id <= BLUE_MAX;
    }

    public static boolean isRedTag(int id) {
      return id >= RED_MIN && id <= RED_MAX;
    }
  }

  // Drive Obstacles (10 rectangles robots cannot drive through)

  public static final Rect[] DRIVE_OBSTACLES = {
    Hub.BLUE_RAMP,
    Hub.RED_RAMP,
    Bump.BLUE_LOWER,
    Bump.BLUE_UPPER,
    Bump.RED_LOWER,
    Bump.RED_UPPER,
    Trench.BLUE_NEAR_PILLAR,
    Trench.BLUE_FAR_PILLAR,
    Trench.RED_NEAR_PILLAR,
    Trench.RED_FAR_PILLAR,
  };

  // Waypoints (named field positions for auto pathfinding, blue alliance)

  public static final class Waypoints {
    // start positions
    public static final Translation2d S_LEFT = new Translation2d(4.51, 0.62);
    public static final Translation2d S_CENTER = new Translation2d(3.55, 4.01);
    public static final Translation2d S_RIGHT = new Translation2d(4.44, 7.42);

    // gateway positions (trench entry/exit)
    public static final Translation2d G_HP_ALLIANCE = new Translation2d(3.50, 0.65);
    public static final Translation2d G_HP_NEUTRAL = new Translation2d(5.50, 0.65);
    public static final Translation2d G_DEPOT_ALLIANCE = new Translation2d(3.50, 7.40);
    public static final Translation2d G_DEPOT_NEUTRAL = new Translation2d(5.50, 7.40);

    // neutral zone
    public static final Translation2d N_OUTPOST = new Translation2d(7.33, 0.77);
    public static final Translation2d N_HP_CENTER = new Translation2d(7.78, 3.01);
    public static final Translation2d N_DEPOT_CENTER = new Translation2d(7.78, 5.06);
    public static final Translation2d N_MID_FIELD = new Translation2d(8.27, 4.03);

    // scoring positions
    public static final Translation2d SCORE_HP = new Translation2d(2.44, 1.80);
    public static final Translation2d SCORE_DEPOT = new Translation2d(2.65, 6.27);

    // supply positions
    public static final Translation2d SUPPLY_HP = new Translation2d(0.49, 0.51);
    public static final Translation2d SUPPLY_DEPOT = new Translation2d(0.69, 5.96);
    public static final Translation2d SUPPLY_OUTPOST_FLOOR = new Translation2d(0.80, 1.50);
    public static final Translation2d SUPPLY_HUB_EXIT = new Translation2d(5.30, 4.03);
  }

  // Zone Helpers

  /** Check if a position is inside the blue alliance zone (X < alliance line). */
  public static boolean isInBlueAllianceZone(Translation2d pos) {
    return pos.getX() < BLUE_ALLIANCE_LINE_X;
  }

  /** Check if a position is inside the red alliance zone (X > red alliance line). */
  public static boolean isInRedAllianceZone(Translation2d pos) {
    return pos.getX() > RED_ALLIANCE_LINE_X;
  }

  /** Check if a position is in the alliance zone for the given color. */
  public static boolean isInAllianceZone(Translation2d pos, boolean isBlue) {
    return isBlue ? isInBlueAllianceZone(pos) : isInRedAllianceZone(pos);
  }

  /** Check if a position is in the neutral zone (between both alliance zones). */
  public static boolean isInNeutralZone(Translation2d pos) {
    return pos.getX() >= BLUE_ALLIANCE_LINE_X && pos.getX() <= RED_ALLIANCE_LINE_X;
  }

  /** Flip a blue-side position to the red side (180-degree rotational symmetry). */
  public static Translation2d flipToRed(Translation2d bluePos) {
    return new Translation2d(FIELD_LENGTH_M - bluePos.getX(), FIELD_WIDTH_M - bluePos.getY());
  }

  // Rect: axis-aligned bounding box utility

  public record Rect(double minX, double minY, double maxX, double maxY) {

    public Translation2d center() {
      return new Translation2d((minX + maxX) / 2.0, (minY + maxY) / 2.0);
    }

    public double width() {
      return maxX - minX;
    }

    public double height() {
      return maxY - minY;
    }

    public boolean contains(Translation2d point) {
      return point.getX() >= minX
          && point.getX() <= maxX
          && point.getY() >= minY
          && point.getY() <= maxY;
    }

    public boolean contains(double x, double y) {
      return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    /** Grow (or shrink if negative) all edges by the given margin. */
    public Rect inflatedBy(double margin) {
      return new Rect(minX - margin, minY - margin, maxX + margin, maxY + margin);
    }

    /** Create a Rect centered on (cx, cy) with the given width and height. */
    public static Rect fromCenter(double cx, double cy, double w, double h) {
      return new Rect(cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0);
    }

    /** The 4 corners, useful for visibility graph node generation. */
    public Translation2d[] corners() {
      return new Translation2d[] {
        new Translation2d(minX, minY),
        new Translation2d(maxX, minY),
        new Translation2d(maxX, maxY),
        new Translation2d(minX, maxY),
      };
    }

    /** Check if a line segment from p1 to p2 intersects this rectangle (slab method). */
    public boolean intersectsSegment(Translation2d p1, Translation2d p2) {
      double dx = p2.getX() - p1.getX();
      double dy = p2.getY() - p1.getY();

      double tMinX, tMaxX;
      if (Math.abs(dx) < 1e-12) {
        if (p1.getX() < minX || p1.getX() > maxX) return false;
        tMinX = Double.NEGATIVE_INFINITY;
        tMaxX = Double.POSITIVE_INFINITY;
      } else {
        double t1 = (minX - p1.getX()) / dx;
        double t2 = (maxX - p1.getX()) / dx;
        tMinX = Math.min(t1, t2);
        tMaxX = Math.max(t1, t2);
      }

      double tMinY, tMaxY;
      if (Math.abs(dy) < 1e-12) {
        if (p1.getY() < minY || p1.getY() > maxY) return false;
        tMinY = Double.NEGATIVE_INFINITY;
        tMaxY = Double.POSITIVE_INFINITY;
      } else {
        double t1 = (minY - p1.getY()) / dy;
        double t2 = (maxY - p1.getY()) / dy;
        tMinY = Math.min(t1, t2);
        tMaxY = Math.max(t1, t2);
      }

      double tEnter = Math.max(tMinX, tMinY);
      double tExit = Math.min(tMaxX, tMaxY);

      return tEnter <= tExit && tExit >= 0 && tEnter <= 1;
    }
  }
}
