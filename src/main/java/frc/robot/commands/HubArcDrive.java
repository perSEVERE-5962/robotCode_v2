// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import static frc.robot.Constants.HubScoringConstants.BLUE_SCORING_SIDE;
import static frc.robot.Constants.HubScoringConstants.RED_SCORING_SIDE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.HubScoringUtil;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import static frc.robot.Constants.HubScoringConstants.BLUE_SCORING_SIDE;
import static frc.robot.Constants.HubScoringConstants.RED_SCORING_SIDE;
import edu.wpi.first.math.util.Units;
import static frc.robot.Constants.MotorConstants;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.math.geometry.Pose2d;

public class HubArcDrive extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier strafeInput;
  private final Translation2d hubCenter;
  private final double scoringDistance;
  private  Shooter shooter;
  private final Rotation2d scoringSide;
  private Indexer indexer;
  private static double headingError;
  //Command for drive mode where robot orbits around hub, maintaining distance and holding its rotation towards the center of hub,
  //added compensation for robot velocity, discussed in https://www.chiefdelphi.com/t/a-note-on-optimal-driving-for-shoot-on-the-move/512979/11
  public HubArcDrive(SwerveSubsystem swerve, DoubleSupplier strafeInput, 
                     Translation2d hubCenter, double scoringDistance,
                     Rotation2d scoringSide) {
    this.swerve = swerve;
    this.strafeInput = strafeInput;
    this.hubCenter = hubCenter;
    this.scoringDistance = scoringDistance;
    this.shooter = Shooter.getInstance();
    this.scoringSide = scoringSide;
    indexer= Indexer.getInstance();
    
    addRequirements(swerve, shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    // joystick input for movement
    double strafeStrength = strafeInput.getAsDouble();

    // current robot pose and rotation
    Translation2d robotPos = swerve.getPose().getTranslation();
    Rotation2d currentHeading = swerve.getHeading();

    // position relative to hub
    Translation2d hubToRobot = robotPos.minus(hubCenter);
    double distanceFromHub = hubToRobot.getNorm();
    double angleFromHub = Math.atan2(hubToRobot.getY(), hubToRobot.getX());

    // Check arc limits
    double centerAngle = scoringSide.getRadians();
    double halfArcWidth = Math.toRadians(90 / 2.0);
    double angleDifference = angleFromHub - centerAngle;
    angleDifference = Math.atan2(Math.sin(angleDifference), Math.cos(angleDifference));

    boolean atLeftLimit = angleDifference >= halfArcWidth;
    boolean atRightLimit = angleDifference <= -halfArcWidth;
    if (atLeftLimit && strafeStrength > 0) {
      strafeStrength = 0;
    }
    if (atRightLimit && strafeStrength < 0) {
      strafeStrength = 0;
    }

    // get desired distance vs real distance
    double distanceError = distanceFromHub - scoringDistance;
    double radialSpeed = -distanceError * 1.5; // p controller to keep distance
    radialSpeed = MathUtil.clamp(radialSpeed, -1.0, 1.0); // get speed

    if (Math.abs(distanceError) < 0.05) {
      radialSpeed = 0;
    }
    //movement around the arc
    double tangentialSpeed = strafeStrength;
    
    // get x and y from distance to hub
    double radialX = Math.cos(angleFromHub);
    double radialY = Math.sin(angleFromHub);
    // get x and y for around the hub
    double tangentialX = -Math.sin(angleFromHub);
    double tangentialY = Math.cos(angleFromHub);

    double fieldVx = radialSpeed * radialX + tangentialSpeed * tangentialX;
    double fieldVy = radialSpeed * radialY + tangentialSpeed * tangentialY;
    double maxX = MathUtil.clamp(fieldVx,-swerve.getSwerveDrive().getMaximumChassisVelocity() *0.7,swerve.getSwerveDrive().getMaximumChassisVelocity()*0.7);//clamp speeds for controlled turning
    double maxY = MathUtil.clamp(fieldVy,-swerve.getSwerveDrive().getMaximumChassisVelocity() *0.7,swerve.getSwerveDrive().getMaximumChassisVelocity()*0.7);//clamp speeds for controlled turning

   
  
    
 //get robot velocity
    ChassisSpeeds robotVelocity = swerve.getFieldVelocity();
    Translation2d robotVel = new Translation2d(
        robotVelocity.vxMetersPerSecond,
        robotVelocity.vyMetersPerSecond
    );
    //get shooter velocity
    
    // Time for game piece to reach hub
    double timeToHub =  1.14;//1.16 sec
    
    // velocity drift compensation, calculates distance the ball would drift due to do robot velocity, and calculated the new target, factoring in said drift.
    Translation2d velocityDrift = robotVel.times(timeToHub);
    Translation2d compensatedTarget = hubCenter.minus(velocityDrift);
    Translation2d toCompensatedTarget = compensatedTarget.minus(robotPos);
    Rotation2d compensatedAim = toCompensatedTarget.getAngle();
    //compare desired heading vs wanted heading
    // Heading control with velocity compensation, to compensate for the velocity of ball due to robot speed
    headingError = compensatedAim.minus(currentHeading).getRadians();
    headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
    double headingSpeed = headingError*4.5;//tune pid
    headingSpeed += tangentialSpeed / scoringDistance;
    double maxHeadingSpeed = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();
    headingSpeed = MathUtil.clamp(headingSpeed, -maxHeadingSpeed*0.4, maxHeadingSpeed*0.4);//clamp speeds for controlled turning
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        maxX, maxY, headingSpeed, currentHeading
    );

    swerve.drive(speeds);
    //System.out.println(headingError);
    //System.out.println("velocity:"+robotVelocity.vyMetersPerSecond);
    double shooterSpeed=Constants.MotorConstants.DESIRED_SHOOTER_RPM+( Math.abs(robotVelocity.vyMetersPerSecond*350));
    shooterSpeed = MathUtil.clamp(shooterSpeed, 0, 3760);
    shooter.moveToVelocityWithPID(shooterSpeed);
    //System.out.println("shooter speed" + shooterSpeed);
    
    
    }
    
  

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0, 0, 0));
    shooter.move(0);
  }

  @Override
  public boolean isFinished() {
    Pose2d pose = swerve.getPose();
    if (scoringSide == BLUE_SCORING_SIDE && pose.getX() > 4.611) {
      System.out.print("Wrong side");
      return true;

    } else if (scoringSide == RED_SCORING_SIDE && pose.getX() < 11.901424) {
      System.out.print("Wrong side");
      return true;
    }
    return false;
  }

  public static boolean checkHeadingError(){
    if(headingError<.1&&headingError>-0.1){
    return true;      
    }
    else{
      return false;
    }
  }
}
