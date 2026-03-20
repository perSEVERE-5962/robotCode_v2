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
  public static final boolean TUNING_MODE = true;

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
    public static final int kIntakePivotID = 56;
    public static final int kIntakeRollerID = 55;
    public static final int kHangerID = 60;
    public static final int kAgitatorID = 54;
  }

  public static final class MotorConstants {
    public static final double DESIRED_SHOOTER_RPM = 3730;
    public static final double DESIRED_INDEXER_RPM = 7833; // 8.4 * 3730/4
    public static final double OUT_INTAKE_POS = 38.24;
    public static final double IN_INTAKE_POS = 11.6;
    public static final double DESIRED_INTAKE_RPM = 0;
    public static final double UP_HANGER_POS = 0;
    public static final double DOWN_HANGER_POS = 0;
    public static final double DESIRED_AGITATOR_SPEED = .5;
  }

  public static final class IntakeRollerConstants {
    public static final double P = 1.0;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double MinOutput = -1.0;
    public static final double MaxOutput = 1.0;
    public static final double FF = 0.0;
    public static final double Iz = 0.0;
  }

  public static final class IntakePivotConstants {
    public static final double P = 1.0;
    public static final double I = 0.0;
    public static final double D = 0.2;
    public static final double MinOutput = -1.0;
    public static final double MaxOutput = 1.0;
    public static final double FF = 0.0;
    public static final double Iz = 0.0;
  }

  public static final class ShooterConstants {
    // Velocity PID tuning (from Kfir2026)
    public static final double P = 0.00011;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double FF = 0.000172;
    public static final double MinOutput = -1.0;
    public static final double MaxOutput = 1.0;
    public static final double Iz = 0.0;

    // Tuning targets
    public static final double TARGET_RPM = 3730;
    public static final double TARGET_FIRE_RATE_PER_SEC = 2.5;
    public static final double TARGET_RECOVERY_MS = 150.0;

    // Telemetry constants
    public static final double SPEED_TOLERANCE_RPM = 50.0;
    public static final double VELOCITY_CONVERSION = 1.0;
    public static final double SHOT_DETECTION_DROP_RPM = 200.0;
    public static final double SHOT_DETECTION_MIN_RPM = 1000.0;
    public static final double TEMP_WARNING_CELSIUS = 65.0;
  }

  public static final class IndexerConstants {
    // Velocity PID (from Alden)
    public static final double P = 0.000;
    public static final double I = 0.0;
    public static final double D = 0.00;
    public static final double MinOutput = -1.0;
    public static final double MaxOutput = 1.0;
    public static final double FF = 0.0004;
    public static final double Iz = 0.0;

    // Telemetry constants
    public static final double TARGET_SPEED = 7833;
    public static final double JAM_CURRENT_THRESHOLD_AMPS = 35.0;
    public static final double JAM_TIME_THRESHOLD_SECONDS = 0.3;
  }

  public static final class AgitatorConstants {
    public static final double P = 0.000;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double MinOutput = -1.0;
    public static final double MaxOutput = 1.0;
    public static final double FF = 0.0002;
    public static final double Iz = 0.0;

    public static final double TARGET_RPM = 2000;
    public static final double JAM_CURRENT_THRESHOLD_AMPS = 25.0;
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

    // Per-component falling-edge debounce for the 8-condition ReadyToShoot composite.
    // Each condition holds true independently after its raw input drops, so a single-frame
    // glitch on one sensor doesn't kill the whole composite.
    public static final double SHOOTER_READY_DEBOUNCE_SEC = 0.08;
    public static final double INDEXER_CLEAR_DEBOUNCE_SEC = 0.05;
    public static final double VISION_LOCKED_DEBOUNCE_SEC = 0.10;
    public static final double SHOT_CONFIDENT_DEBOUNCE_SEC = 0.08;
    // hasBall and fireAuthorized: no debounce (instant response)
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
      "FrontLeftDrive", // 0
      "FrontLeftTurn", // 1
      "FrontRightDrive", // 2
      "FrontRightTurn", // 3
      "BackLeftDrive", // 4
      "BackLeftTurn", // 5
      "BackRightDrive", // 6
      "BackRightTurn", // 7
      "Shooter", // 8
      "Indexer", // 9
      "Agitator", // 10
      "Intake", // 11
      "IntakeActuator", // 12
      "Hanger", // 13
      "Ch14", // 14 - unused/unknown
      "Ch15", // 15 - unused/unknown
      "Ch16", // 16 - unused/unknown
      "Ch17", // 17 - unused/unknown
      "Ch18", // 18 - unused/unknown
      "Ch19", // 19 - unused/unknown
      "Radio", // 20
      "RoboRIO", // 21
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

  public static final class HubTimingConstants {
    // Fuel processing at the hub (measured from field tour videos + Week 0)
    public static final double FUEL_COUNT_DELAY_MIN_SEC = 1.0;
    public static final double FUEL_COUNT_DELAY_MAX_SEC = 2.0;
    // Hub counts fuel for 3 seconds after active window ends (game manual)
    public static final double FUEL_COUNT_EXTENSION_SEC = 3.0;
    // Margin buffer: shot must land this far inside window for FIRE_AUTHORIZED
    public static final double FIRE_SAFE_MARGIN_SEC = 0.5;
    // How early before active window to trigger PRE_SPIN
    public static final double PRE_SPIN_WINDOW_SEC = 3.0;
    // Extra margin when schedule confidence is FALLBACK (50/50 guess)
    public static final double FALLBACK_MARGIN_EXTRA_SEC = 1.0;
    // How often GAME_DATA_MISSING alert repeats during transition
    public static final double GAME_DATA_ALERT_INTERVAL_SEC = 2.0;

    // Hub shift schedule: elapsed seconds from teleop start
    // Game Manual Section 6.4: 10s transition, 4x25s shifts, 30s endgame = 140s total
    public static final double TRANSITION_END = 10.0;
    public static final double SHIFT1_END = 35.0;
    public static final double SHIFT2_END = 60.0;
    public static final double SHIFT3_END = 85.0;
    public static final double SHIFT4_END = 110.0;
    public static final double ENDGAME_END = 140.0;

    public static final double AUTO_DURATION = 20.0;
  }

  public static final class SpatialLaunchConstants {
    // Closer than this and the ball trajectory is nearly vertical
    public static final double MIN_LAUNCH_DISTANCE_M = 1.0;

    // Ball radius used to expand obstacle AABBs (a near-miss is still a miss)
    public static final double BALL_RADIUS_M = 0.075;

    // Trench pillar AABBs from game manual section 5 field drawings.
    // Each row: {minX, minY, maxX, maxY}. Pillars are ~0.305m square, 1.346m tall.
    public static final double[][] TRENCH_PILLARS = {
      {3.960, 1.265, 4.265, 1.570},
      {3.960, 6.177, 4.265, 6.482},
      {12.276, 1.265, 12.581, 1.570},
      {12.276, 6.177, 12.581, 6.482},
    };

    // Trench ceiling exclusion zones: shooting from under here hits the ceiling.
    // XY rectangles from field geometry (trench ceiling AABBs projected to 2D).
    public static final double[][] TRENCH_EXCLUSION_ZONES = {
      {3.960, 1.570, 5.180, 3.730},
      {3.960, 4.320, 5.180, 6.170},
      {11.361, 1.570, 12.581, 3.730},
      {11.361, 4.320, 12.581, 6.170},
    };

    // Time for the robot to react before entering a no-shoot zone (seconds)
    public static final double RETRACTION_TIME_SEC = 0.25;

    // Heading tolerance for ReadyToShoot (degrees)
    public static final double HEADING_TOLERANCE_DEG = 5.0;
  }
}
