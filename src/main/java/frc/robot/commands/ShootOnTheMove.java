// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.DoubleSupplier;

// Shoot on the move command, this command has very similar heading control to the hub arc drive
// command, where error is calculated, time of flight is considered to compenstae for drift
// and the robot continues to aim to hub, the differe is, the robot is free to move in both the x
// and y directions, this makes time of flight calculations more complicated, and needs to be tuned
// as explained in the thread below, we made a interpolating double tree map, or essentially a look
// up table, this helps calculate differnt time of flight at different distances in both x and y
// must find different distances and times for tuning, the heading aim is essentially the same as
// hub arc drive, the differnce is calculating time of flight, as distance changes, and shooter
// speed is involved, as ballls now are affected by x and y velocity of robot.
// the command iterates to constantly update shot data, imagine the first shot 5 meters away with
// the robot moving sideways, the lookup table says it takes 1 sec to get ther for example
// but since we are moving, we will actually be 6 meters from the hub, so loop iterates and sees
// time of flight is now 1.5 seconds, causing more drift, meaning aim more to compensate, and this
// proccess repeats until the changes are so slight, there is no impact.
// https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815/27
public class ShootOnTheMove extends Command {

  // Telemetry snapshot: static so ShotPredictorTelemetry can read without a reference
  private static volatile boolean active = false;
  private static volatile double snapDistanceToHubM = 0;
  private static volatile double snapTimeOfFlightSec = 0;
  private static volatile double snapCompTargetX = 0;
  private static volatile double snapCompTargetY = 0;
  private static volatile double snapDriftX = 0;
  private static volatile double snapDriftY = 0;
  private static volatile double snapComputedRPM = 0;
  private static volatile double snapHeadingErrorRad = 0;
  private static volatile double snapHeadingSpeedRadPerSec = 0;

  public static boolean isActive() {
    return active;
  }

  public static double getSnapDistanceToHubM() {
    return snapDistanceToHubM;
  }

  public static double getSnapTimeOfFlightSec() {
    return snapTimeOfFlightSec;
  }

  public static double getSnapCompTargetX() {
    return snapCompTargetX;
  }

  public static double getSnapCompTargetY() {
    return snapCompTargetY;
  }

  public static double getSnapDriftX() {
    return snapDriftX;
  }

  public static double getSnapDriftY() {
    return snapDriftY;
  }

  public static double getSnapComputedRPM() {
    return snapComputedRPM;
  }

  public static double getSnapHeadingErrorRad() {
    return snapHeadingErrorRad;
  }

  public static double getSnapHeadingSpeedRadPerSec() {
    return snapHeadingSpeedRadPerSec;
  }

  private static void clearSnapshot() {
    active = false;
    snapDistanceToHubM = 0;
    snapTimeOfFlightSec = 0;
    snapCompTargetX = 0;
    snapCompTargetY = 0;
    snapDriftX = 0;
    snapDriftY = 0;
    snapComputedRPM = 0;
    snapHeadingErrorRad = 0;
    snapHeadingSpeedRadPerSec = 0;
  }

  private final SwerveSubsystem swerve;
  private final Shooter shooter;

  private final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);
  private final DoubleSupplier forwardInput;
  private final DoubleSupplier strafeInput;
  private final Translation2d hubCenter;
private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap RPMMap = new InterpolatingDoubleTreeMap();

  public ShootOnTheMove(SwerveSubsystem swerve,
                        DoubleSupplier forwardInput,
                        DoubleSupplier strafeInput,
                        Translation2d hubCenter) {
    this.swerve = swerve;
    this.shooter = Shooter.getInstance();
    this.forwardInput = forwardInput;
    this.strafeInput = strafeInput;
    this.hubCenter = hubCenter;

    addRequirements(swerve);
    //distance and time
    timeOfFlightMap.put(1.2, 2.0);
    //distance and rpm, more effeicent and consistent than physics for testing and tuning
    timeOfFlightMap.put(2.0, 3.0);
    RPMMap.put(1.2, 3720.0);
    RPMMap.put(2.0, 4000.0);

  }

  @Override
  public void initialize() {
    active = true;
  }

  @Override
  public void execute() {
    Translation2d robotPos = swerve.getPose().getTranslation();
    Rotation2d currentHeading = swerve.getHeading();

    ChassisSpeeds robotVelocity = swerve.getFieldVelocity();
    Translation2d robotVel =
        new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);

    double distanceToHub = robotPos.getDistance(hubCenter);
    System.out.println(distanceToHub);

     double timeOfFlight = timeOfFlightMap.get(distanceToHub);
    double shooterRPM = 0;
    Translation2d compensatedTarget = hubCenter;  // will be refined each iteration
      for (int i = 0; i <= 3; i++) {

      // how far the piece drifts during flight due to robot motion 
      Translation2d drift = robotVel.times(timeOfFlight);
      lastDrift = drift;
      compensatedTarget = hubCenter.minus(drift);
      double distanceToCompensated = robotPos.getDistance(compensatedTarget);

      //get time of flight and rpm to compenate
      timeOfFlight = timeOfFlightMap.get(distanceToCompensated);
      shooterRPM = RPMMap.get(distanceToCompensated);

    }

    Rotation2d targetAngle = compensatedTarget.minus(robotPos).getAngle();
    double headingError = MathUtil.angleModulus(targetAngle.minus(currentHeading).getRadians());
    double headingSpeed = MathUtil.clamp(headingError * 2, -4, 4);
    double maxV  = swerve.getSwerveDrive().getMaximumChassisVelocity();
  
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        forwardInput.getAsDouble() * maxV * 0.6, strafeInput.getAsDouble()  * maxV * 0.6, headingSpeed, currentHeading
    );
    swerve.drive(speeds);
   
    System.out.println(shooterRPM);
    shooter.moveToVelocityWithPID(shooterRPM);

    // Telemetry snapshot
    snapDistanceToHubM = distanceToHub;
    snapTimeOfFlightSec = timeOfFlight;
    snapCompTargetX = compensatedTarget.getX();
    snapCompTargetY = compensatedTarget.getY();
    snapDriftX = lastDrift.getX();
    snapDriftY = lastDrift.getY();
    snapComputedRPM = shooterRPM;
    snapHeadingErrorRad = headingError;
    snapHeadingSpeedRadPerSec = headingSpeed;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0, 0, 0));
    clearSnapshot();
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
