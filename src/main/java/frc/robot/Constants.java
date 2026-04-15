// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /**
   * Master tuning mode switch. - true: Values read from dashboard (practice/testing) - false:
   * Values locked to defaults (competition)
   *
   * <p>IMPORTANT: Set to FALSE before competition deployment!
   */
  public static final boolean TUNING_MODE = false;

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class FieldConstants {
    public static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final double DistanceToHub = 0.4;
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class HubScoringConstants {

    // Hub positions on field
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.625594, 4.0);
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.901424, 4.0);

    // Scoring parameters
    public static final double SCORING_DISTANCE = 2; // meters from hub center
    public static final double MAX_ARC_SPEED = 2.0; // max speed while driving along arc (m/s)

    // Valid scoring arc definition
    public static final Rotation2d BLUE_SCORING_SIDE = Rotation2d.fromDegrees(180); // Faces +X
    public static final Rotation2d RED_SCORING_SIDE = Rotation2d.fromDegrees(0); // Faces -X
    public static final double SCORING_ARC_WIDTH_DEGREES =
        90; // 180 = semicircle, 90 = quarter circle

    // Tolerances
    public static final double POSITION_TOLERANCE = 0.05; // meters
    public static final double ANGLE_TOLERANCE = 2.0; // degrees
  }

  /**
   * Teleop assist: poses the robot drives to when the copilot holds a button. All poses are in BLUE
   * alliance coordinates. pathfindToPoseFlipped handles red auto-flip. Rotation = the heading the
   * robot faces on arrival. AimAndShoot overrides it for shooting. TUNE THESE on the real field,
   * especially the intake spots.
   */
  public static final class TeleopAssistConstants {
    // scoring spots: 2.5m from blue hub center (4.5974, 4.035), 45 deg off the alliance-facing
    // direction. Two spots symmetric about the hub, one toward HP (low Y), one toward depot
    // (high Y). Heading points the front away from the hub so the rear shooter faces it.
    // AimAndShoot corrects the heading after arrival anyway.
    public static final double SCORING_RADIUS_M = 2.5;
    public static final Pose2d BLUE_HP_SCORING_POSE =
        new Pose2d(2.830, 2.267, Rotation2d.fromDegrees(45));
    public static final Pose2d BLUE_DEPOT_SCORING_POSE =
        new Pose2d(2.830, 5.803, Rotation2d.fromDegrees(-45));

    // intake spots: near the corral (HP side) and mirrored depot side.
    // HP corral is at X [1.80, 2.71], Y [0, 0.96]. Robot sits just outside facing in.
    // Depot side is mirrored across the hub's Y center line.
    // heading 0 = facing +X (into the field) so the front intake picks up balls
    public static final Pose2d BLUE_HP_INTAKE_POSE =
        new Pose2d(2.26, 1.45, Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_DEPOT_INTAKE_POSE =
        new Pose2d(2.26, 6.62, Rotation2d.fromDegrees(0));

    // how fast the robot drives during teleop assist (slower = safer, driver can react)
    public static final double ASSIST_MAX_VEL_MPS = 3.0;
    public static final double ASSIST_MAX_ACCEL_MPSS = 2.0;
    public static final double ASSIST_MAX_ANGULAR_VEL_RADPS = Math.toRadians(540);
    public static final double ASSIST_MAX_ANGULAR_ACCEL_RADPSS = Math.toRadians(720);
  }

  public static class PhotonvisionConstants {

    /*
    deploy order for Json files of swerve modules
    frontleft,
    frontright,
    backleft,
    backright
     */
  }

  public static final class CANDeviceIDs {
    public static final int kIndexerID = 50;
    public static final int kShooterID = 52;
    public static final int kShooterFollower = 53;
    public static final int kShooterFollower1 = 54;
    public static final int kShooterFollower2 = 55;
    public static final int kIntakePivotID = 56;
    public static final int kIntakeRollerID = 57;
    public static final int kHangerID = 60;
    public static final int kAgitatorID = 51;
  }

  public static final class MotorConstants {
    public static final double DESIRED_SHOOTER_RPM = 2500;
    public static final double DESIRED_INDEXER_RPM = 6000; // 8.4 * 3730/4
    public static final double OUT_INTAKE_POS = 0.0;
    public static final double IN_INTAKE_POS = 0.3;
    public static final double DESIRED_INTAKE_RPM = 5000;
    public static final double UP_HANGER_POS = 0;
    public static final double DOWN_HANGER_POS = 0;
    public static final double DESIRED_AGITATOR_SPEED = 5990;
  }

  public static final class IntakeRollerConstants {
    public static final double P = 0.0001;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double MinOutput = -1.0;
    public static final double MaxOutput = 1.0;
    public static final double FF = 0.02;
    public static final double Iz = 0.0;
  }

  public static final class IntakePivotConstants {
    public static final double kG = 0.15;
    public static final double P = 54; // 1.0
    public static final double I = 0.0;
    public static final double D = 0.00; // .2
    public static final double MinOutput = -1.0;
    public static final double MaxOutput = 1.0;
    public static final double FF = 0.0;
    public static final double Iz = 0.0;
  }

  public static final class ShooterConstants {
    // Velocity PID tuning (from Kfir2026)
    public static final double P = 0.00009;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double FF = 0.000189; // .000172
    public static final double MinOutput = -1.0;
    public static final double MaxOutput = 1.0;
    public static final double Iz = 0.0;

    // Tuning targets
    public static final double TARGET_RPM = 2500;
    public static final double TARGET_FIRE_RATE_PER_SEC = 2.5;
    public static final double TARGET_RECOVERY_MS = 150.0;
    public static final double RPM_SLEW_RATE = 1000.0; // max change in RPM per second

    // Telemetry constants
    public static final double SPEED_TOLERANCE_RPM = 150.0;
    public static final double VELOCITY_CONVERSION = 1.0;
    public static final double SHOT_DETECTION_DROP_RPM = 200.0;
    public static final double SHOT_DETECTION_MIN_RPM = 1000.0;
    public static final double TEMP_WARNING_CELSIUS = 65.0;
    public static final double EMERGENCY_DUMP_RPM = 2500.0;
  }

  public static final class IndexerConstants {
    // Velocity PID (from Alden)
    public static final double P = 0.0001;
    public static final double I = 0.0;
    public static final double D = 0.00;
    public static final double MinOutput = -1.0;
    public static final double MaxOutput = 1.0;
    public static final double FF = 0.000154; // .0004
    public static final double Iz = 0.0;

    // Telemetry constants
    public static final double TARGET_SPEED = 6000;
    public static final double JAM_CURRENT_THRESHOLD_AMPS = 70;
    public static final double JAM_TIME_THRESHOLD_SECONDS = 0.3;
  }

  public static final class AgitatorConstants {
    public static final double P = 0.1;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double MinOutput = -1.0;
    public static final double MaxOutput = 1.0;
    public static final double FF = 0.016;
    public static final double Iz = 0.0;

    public static final double TARGET_RPM = 5676;
    public static final double JAM_CURRENT_THRESHOLD_AMPS = 79;
    public static final double JAM_TIME_THRESHOLD_SECONDS = 0.3;
  }

  public static final class HopperConstants {
    public static final int LOW_BALL_THRESHOLD = 3;
    public static final double DETECTION_CONFIDENCE_THRESHOLD = 0.8;
  }

  public static final class HangerConstants {
    public static final double P = 1.0;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double MinOutput = -1.0;
    public static final double MaxOutput = 1.0;
    public static final double FF = 0.0;
    public static final double Iz = 0.0;
    public static final double POSITION_TOLERANCE = 0.1;
  }

  public static final class BatteryThresholds {
    public static final double CRITICAL_V = 10.0;
    public static final double WARNING_V = 11.5;
    public static final double PRE_MATCH_WARN_V = 12.3;
    public static final double PRE_MATCH_FAIL_V = 12.0;
  }

  /** CAN bus disconnect debounce and scoring readiness stabilization */
  public static final class DeviceHealthConstants {
    // Motor must report disconnected for this long before we flag it (filters CAN glitches)
    public static final double DISCONNECT_DEBOUNCE_SEC = 0.5;
    // ReadyToShoot stays true through brief velocity dips during sustained fire
    public static final double READY_TO_SHOOT_DEBOUNCE_SEC = 0.15;
  }

  /**
   * Pre-deploy and pre-merge safety gates. Run via Gradle tasks:
   *
   * <ul>
   *   <li>{@code ./gradlew checkDeploy} blocks deploy when TUNING_MODE is on
   *   <li>{@code ./gradlew checkCompetition} stricter gate for match day
   * </ul>
   *
   * Skip with {@code ./gradlew deploy -x checkDeploy} if you really need tuning on the robot.
   */
  public static final class DeploySafetyCheck {
    /** Returns false if TUNING_MODE is on (unsafe for competition deploy). */
    public static boolean isSafeForDeploy() {
      return !TUNING_MODE;
    }

    // back left left 11.54, 11.54, 45 degrees 15 desgrees
    /** Gradle entry point: exits 1 if deploy is unsafe. */
    public static void main(String... args) {
      if (!isSafeForDeploy()) {
        System.err.println("DEPLOY BLOCKED: TUNING_MODE = true");
        System.err.println("Set Constants.TUNING_MODE = false before deploying to robot.");
        System.err.println("Override: ./gradlew deploy -x checkDeploy");
        System.exit(1);
      }
      System.out.println("Deploy safety check passed.");
    }
  }

  public static final class LEDConstants {
    public static final int PWM_PORT = 0;
    public static final int STRIP_LENGTH = 19;
    public static final double DIM_DISABLED = 0.15;
  }

  /**
   * PDH channel map. Maps physical PDH channel (0-23) to a human-readable circuit name. Update this
   * when wiring changes. Channels without a label are logged as "Ch{N}".
   */
  public static final class PDHChannelMap {
    public static final int NUM_CHANNELS = 24;

    // Channel labels: index = PDH channel number, value = circuit name
    // Update these to match your actual wiring harness
    private static final String[] LABELS = {
      "FrontLeftDrive", // 0 not confident
      "FrontLeftTurn", // 1 not confident
      "FrontRightDrive", // 2 not confident
      "FrontRightTurn", // 3 not confident
      "BackLeftDrive", // 4 not confident
      "IntakePivot", // 5
      "Shooter", // 6
      "Shooter", // 7
      "Ch8", // 8 unused/unknown
      "Agitator", // 9
      "Indexer", // 10
      "Shooter", // 11
      "Shooter", // 12
      "Indexer", // 13
      "Ch14", // 14 - unused/unknown
      "MPM", // 15 -
      "Ch16", // 16 - unused/unknown
      "Ch17", // 17 - unused/unknown
      "Ch18", // 18 - unused/unknown
      "Ch19", // 19 - unused/unknown
      "Radio", // 20 not confident
      "RoboRIO", // 21 not confident
      "Ch22", // 22 - unused/unknown
      "Ch23", // 23 - unused/unknown
    };

    /** Alert when any single channel exceeds this current (amps). */
    public static final double CHANNEL_OVERCURRENT_AMPS = 40.0;

    public static String getLabel(int channel) {
      if (channel >= 0 && channel < LABELS.length) {
        return LABELS[channel];
      }
      return "Ch" + channel;
    }
  }

  /** Jam protection: 3-layer debounce + auto-reverse + disable after max attempts */
  public static final class JamProtectionConstants {
    // Intake jam protection
    public static final double INTAKE_JAM_CURRENT_AMPS = 25.0;
    public static final double INTAKE_JAM_VELOCITY_RPM = 100.0;
    public static final double INTAKE_STARTUP_IGNORE_SEC = 0.5;
    public static final double INTAKE_JAM_CONFIRM_SEC = 0.3;
    public static final double INTAKE_REVERSE_SEC = 0.4;
    public static final double INTAKE_COOLDOWN_SEC = 0.15;
    public static final double INTAKE_REVERSE_POWER = -0.4;
    public static final int INTAKE_MAX_ATTEMPTS = 3;

    // Indexer jam protection (raised confirm + velocity to avoid false triggers during ball
    // passage)
    public static final double INDEXER_JAM_CURRENT_AMPS = 25.0;
    public static final double INDEXER_JAM_VELOCITY_RPM = 200.0;
    public static final double INDEXER_STARTUP_IGNORE_SEC = 0.5;
    public static final double INDEXER_JAM_CONFIRM_SEC = 0.5;
    public static final double INDEXER_REVERSE_SEC = 0.3;
    public static final double INDEXER_COOLDOWN_SEC = 0.15;
    public static final double INDEXER_REVERSE_POWER = -0.3;
    public static final int INDEXER_MAX_ATTEMPTS = 3;

    // Agitator jam protection (raised current threshold, needs real stall current measurement)
    public static final double AGITATOR_JAM_CURRENT_AMPS = 20.0;
    public static final double AGITATOR_JAM_VELOCITY_RPM = 100.0;
    public static final double AGITATOR_STARTUP_IGNORE_SEC = 0.5;
    public static final double AGITATOR_JAM_CONFIRM_SEC = 0.3;
    public static final double AGITATOR_REVERSE_SEC = 0.4;
    public static final double AGITATOR_COOLDOWN_SEC = 0.15;
    public static final double AGITATOR_REVERSE_POWER = -0.3;
    public static final int AGITATOR_MAX_ATTEMPTS = 3;
  }

  /** Stall = high current + low velocity for debounce time */
  public static final class StallDetectionConstants {
    // Shooter stall thresholds
    public static final double SHOOTER_STALL_CURRENT_AMPS = 50.0;
    public static final double SHOOTER_STALL_VELOCITY_RPM = 100.0;
    public static final double SHOOTER_STALL_DEBOUNCE_MS = 250.0;

    // Indexer stall thresholds
    public static final double INDEXER_STALL_CURRENT_AMPS = 15.0;
    public static final double INDEXER_STALL_VELOCITY_RPM = 50.0;
    public static final double INDEXER_STALL_DEBOUNCE_MS = 200.0;

    // Intake stall thresholds
    public static final double INTAKE_STALL_CURRENT_AMPS = 20.0;
    public static final double INTAKE_STALL_VELOCITY_RPM = 100.0;
    public static final double INTAKE_STALL_DEBOUNCE_MS = 200.0;
  }

  /** Projectile physics constants (measured from CAD) */
  public static final class ProjectileSimConstants {
    public static final double EXIT_HEIGHT_M = 0.4313; // 16.98in from floor
    public static final double WHEEL_DIAMETER_M = 0.1016; // 4" flywheel
    public static final double TARGET_HEIGHT_M = 1.83; // 72in hub entry
    public static final double DT = 0.001; // integration step (s)
    public static final double DRAG_COEFFICIENT = 0.47; // smooth sphere
    public static final double BALL_MASS_KG = 0.27; // ~9.5oz fuel ball
    public static final double BALL_RADIUS_M = 0.0889; // 3.5" radius
  }

  /** Shot calculator constants (launcher geometry + Newton solver + SOTM) */
  public static final class ShotCalculatorConstants {
    // Launcher geometry (drum shooter, rear-mounted)
    public static final double LAUNCHER_OFFSET_X = -0.0577; // behind center (OnShape)
    public static final double LAUNCHER_OFFSET_Y = 0.0;
    public static final double FIXED_LAUNCH_ANGLE_DEG = 60.0; // drum shooter angle
    // rear-mounted shooter: rotate aim by 180 deg so the back faces the hub
    public static final double SHOOTER_ANGLE_OFFSET_RAD = Math.PI;

    // Newton solver
    public static final int MAX_ITERATIONS = 25;
    public static final double CONVERGENCE_TOLERANCE = 0.001;
    public static final double TOF_MIN = 0.05;
    public static final double TOF_MAX = 5.0;

    // Velocity filter
    public static final double MIN_SOTM_SPEED = 0.1;

    // Latency compensation
    public static final double PHASE_DELAY_MS = 30.0;
    public static final double MECH_LATENCY_MS = 20.0;

    // Drag compensation
    public static final double SOTM_DRAG_COEFF = 0.47;

    // Scoring range
    public static final double MIN_SCORING_DISTANCE = 0.5;
    public static final double MAX_SCORING_DISTANCE = 5.0;

    // Speed cap while shooting on the move
    public static final double MAX_SOTM_SPEED = 3.0;

    // Per-distance RPM band boundaries (meters)
    public static final double RPM_BAND_SHORT_END = 2.2;
    public static final double RPM_BAND_MEDIUM_END = 3.0;

    // Copilot D-pad RPM trim limits
    public static final double RPM_OFFSET_MAX = 200.0;

    // Confidence scoring: heading accuracy scaling
    public static final double HEADING_MAX_ERROR_RAD = Math.toRadians(15);
    public static final double HEADING_SPEED_SCALAR = 1.0;
    public static final double HEADING_REFERENCE_DISTANCE = 2.5;

    // Confidence scoring: component weights (geometric mean)
    public static final double W_CONVERGENCE = 1.0;
    public static final double W_VELOCITY_STABILITY = 0.8;
    public static final double W_VISION_CONFIDENCE = 1.2;
    public static final double W_HEADING_ACCURACY = 1.5;
    public static final double W_DISTANCE_IN_RANGE = 0.5;

    // Directional polar speed limiting
    public static final double MAX_POLAR_ANGULAR_RATE_RAD_PER_SEC = 2.0;
    public static final double POLAR_SPEED_FLOOR_MPS = 0.5;

    // Asymmetric heading tolerance (tight strafing, loose approaching)
    public static final double CROSS_TRACK_TOLERANCE_DEG = 3.0;
    public static final double ALONG_TRACK_TOLERANCE_DEG = 8.0;
  }

  /** Hub shift timing: shift schedule, fuel delay, fire authorization margins */
  public static final class HubTimingConstants {
    public static final double FUEL_COUNT_DELAY_MIN_SEC = 1.0;
    public static final double FUEL_COUNT_DELAY_MAX_SEC = 2.0;
    public static final double FUEL_COUNT_EXTENSION_SEC = 3.0;
    public static final double FIRE_SAFE_MARGIN_SEC = 0.5;
    public static final double PRE_SPIN_WINDOW_SEC = 3.0;
    public static final double FALLBACK_MARGIN_EXTRA_SEC = 1.0;
    public static final double GAME_DATA_ALERT_INTERVAL_SEC = 2.0;

    // Game Manual Section 6.4: 10s transition, 4x25s shifts, 30s endgame = 140s total
    public static final double TRANSITION_END = 10.0;
    public static final double SHIFT1_END = 35.0;
    public static final double SHIFT2_END = 60.0;
    public static final double SHIFT3_END = 85.0;
    public static final double SHIFT4_END = 110.0;
    public static final double ENDGAME_END = 140.0;

    public static final double AUTO_DURATION = 20.0;
  }

  /** Heading lock controller for SOTM: PID + feedforward + lookahead */
  public static final class HeadingLockConstants {
    public static final double P = 2.6;
    public static final double I = 0.0;
    public static final double D = 0.1;

    // FF gains (layer OFF by default, these just sit ready)
    public static final double FF_KS = 0.05;
    public static final double FF_KV = 0.8;
    public static final double FF_KA = 0.0;

    // Lookahead (disabled by default)
    public static final double LOOKAHEAD_SEC = 0.0;

    // FF distance scaling
    public static final double FF_MIN_DIST_M = 0.5;
    public static final double FF_MAX_DIST_M = 1.5;
  }
}
