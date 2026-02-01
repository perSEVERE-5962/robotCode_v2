// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.util.Units;
 import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.function.DoubleSupplier;
//Shoot on the move command, this command has very similar heading control to the hub arc drive command, where error is calculated, time of flight is considered to compenstae for drift
//and the robot continues to aim to hub, the differe is, the robot is free to move in both the x and y directions, this makes time of flight calculations more complicated, and needs to be tuned
//as explained in the thread below, we made a interpolating double tree map, or essentially a look up table, this helps calculate differnt time of flight at different distances in both x and y
//must find different distances and times for tuning, the heading aim is essentially the same as hub arc drive, the differnce is calculating time of flight, as distance changes, and shooter speed is involved, as ballls now are affected by x and y velocity of robot.
//the command iterates to constantly update shot data, imagine the first shot 5 meters away with the robot moving sideways, the lookup table says it takes 1 sec to get ther for example
//but since we are moving, we will actually be 6 meters from the hub, so loop iterates and sees time of flight is now 1.5 seconds, causing more drift, meaning aim more to compensate, and this proccess repeats until the changes are so slight, there is no impact.
//https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815/27
public class ShootOnTheMove extends Command {

  private final SwerveSubsystem swerve;
  private final Shooter shooter;

 private final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);
  private final DoubleSupplier forwardInput;  
  private final DoubleSupplier strafeInput;   
  private final Translation2d hubCenter;
private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
  
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
    timeOfFlightMap.put(2.0, 0.40);
    timeOfFlightMap.put(4.0, 0.75);
    timeOfFlightMap.put(6.0, 1.10);
  }
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    Translation2d robotPos = swerve.getPose().getTranslation();
    Rotation2d currentHeading = swerve.getHeading();

    ChassisSpeeds robotVelocity = swerve.getFieldVelocity();
    Translation2d robotVel = new Translation2d(
        robotVelocity.vxMetersPerSecond,
        robotVelocity.vyMetersPerSecond
    );

    double distanceToHub = robotPos.getDistance(hubCenter);


     double timeOfFlight = timeOfFlightMap.get(distanceToHub);

    Translation2d compensatedTarget = hubCenter;  // will be refined each iteration
    Translation2d shooterVecResult  = Translation2d.kZero;
    double targetShooterSpeed = 0.0;    
      for (int i = 0; i < 3; i++) {

      // how far the piece drifts during flight due to robot motion 
      Translation2d drift = robotVel.times(timeOfFlight);

      //compensated target, pretty much creates a new target, and then with the drift, the ball will end up in the hub
      compensatedTarget = hubCenter.minus(drift);

      //vector from robot to compensated target
      Translation2d toTarget = compensatedTarget.minus(robotPos);
      double distToCompTarget = toTarget.getNorm();

      timeOfFlight = timeOfFlightMap.get(distToCompTarget);
      // required field-relative velocity of the piece
      // The piece must travel toTarget in timeOfFlight seconds.
      Translation2d requiredFieldVel = toTarget.times(1.0 / timeOfFlight);

      //what the shooter itself must produce
     
      shooterVecResult = requiredFieldVel.minus(robotVel);
      targetShooterSpeed = shooterVecResult.getNorm();

      // shooter speed should never be zero or negative for division
      if (targetShooterSpeed < 0.1) {
        targetShooterSpeed = 10;
      }


    }

    Rotation2d compensatedAim = compensatedTarget.minus(robotPos).getAngle();

    double headingError = compensatedAim.minus(currentHeading).getRadians();

    headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

    double headingSpeed = headingError * 3;// tune pid
    double maxHeadingSpeed = swerve.getSwerveDrive().getMaximumChassisAngularVelocity() * 0.6;
    headingSpeed = MathUtil.clamp(headingSpeed, -maxHeadingSpeed, maxHeadingSpeed);

    double maxV  = swerve.getSwerveDrive().getMaximumChassisVelocity();
    double fieldVx = forwardInput.getAsDouble() * maxV * 0.6;
    double fieldVy = strafeInput.getAsDouble()  * maxV * 0.6;


    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldVx, fieldVy, headingSpeed, currentHeading
    );
    swerve.drive(speeds);
    //double totalLaunchSpeed = targetShooterSpeed / Math.cos(ShooterAngle!!!);
    double shooterRPM = (targetShooterSpeed / (2.0 * Math.PI * WHEEL_RADIUS_METERS)) * 60.0;//replace with total launch speed
    shooter.moveToVelocityWithPID(shooterRPM);
 
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0, 0, 0));

  }

  @Override
  public boolean isFinished() {
  
    return false;
  }
}