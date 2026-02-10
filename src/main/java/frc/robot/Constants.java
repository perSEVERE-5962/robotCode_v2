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
    public static final double SCORING_DISTANCE = 1.06; // meters from hub center
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
    public static final int kIntakeActuatorID = 56;
    public static final int kIntakeID = 55;
    public static final int kHangerID = 992;
  }

  public static final class MotorConstants {
    public static final double DESIRED_SHOOTER_RPM = 3730;
    public static final double DESIRED_INDEXER_RPM = 8.4 * 3730 / 4;
    public static final double OUT_INTAKE_POS = 3.0;
    public static final double IN_INTAKE_POS = 0.0;
    public static final double DESIRED_INTAKE_SPEED = 1;
    public static final double UP_HANGER_POS = 50.0;
    public static final double DOWN_HANGER_POS = 0.0;
  }

  public static final class IntakeConstants {
    public static final double P = 1.0;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double MinOutput = -1.0;
    public static final double MaxOutput = 1.0;
    public static final double FF = 0.0;
    public static final double Iz = 0.0;
  }

  public static final class ShooterConstants {
    // Velocity PID tuning (from Kfir2026)
    public static final double P = 0.0001;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double FF = 0.0000145;
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
    public static final double P = 0.00011;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double MinOutput = -1.0;
    public static final double MaxOutput = 1.0;
    public static final double FF = 0.000014;
    public static final double Iz = 0.0;

    // Telemetry constants
    public static final double TARGET_SPEED = 0.4;
    public static final double JAM_CURRENT_THRESHOLD_AMPS = 25.0;
    public static final double JAM_TIME_THRESHOLD_SECONDS = 0.25;
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
}
