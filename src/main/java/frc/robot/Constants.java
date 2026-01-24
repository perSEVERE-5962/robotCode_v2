// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }
  public static final class FieldConstants {
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(
      AprilTagFields.k2026RebuiltAndymark);

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
  
    // Hub positions on field (adjust based on your field layout)
    public static final Translation2d RED_HUB_CENTER = new Translation2d(4.625594, 4.0);
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(11.901424, 4.0); // Adjust for red side
  
    // Scoring parameters
    public static final double SCORING_DISTANCE = 2.0; // meters from hub center
    public static final double MAX_ARC_SPEED = 2.0; // max speed while driving along arc (m/s)
  
    // Valid scoring arc definition
    public static final Rotation2d BLUE_SCORING_SIDE = Rotation2d.fromDegrees(0); // Faces +X
    public static final Rotation2d RED_SCORING_SIDE = Rotation2d.fromDegrees(180); // Faces -X
    public static final double SCORING_ARC_WIDTH_DEGREES = 90; // 180 = semicircle, 90 = quarter circle
  
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
    public static final int kIntakeActuatorID = 999;
    public static final int kIntakeID = 999;
  }

  public static final class MotorConstants {
    public static final double DESIRED_SHOOTER_SPEED = 1.0;
    public static final double DESIRED_INDEXER_SPEED = 0.5;
    public static final double OUT_INTAKE_POS = 999;
    public static final double IN_INTAKE_POS = 999;
    public static final double DESIRED_INTAKE_SPEED = 999;
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
}
