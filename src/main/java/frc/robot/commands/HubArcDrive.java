// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.HubScoringUtil;

import static frc.robot.Constants.HubScoringConstants.BLUE_SCORING_SIDE;
import static frc.robot.Constants.HubScoringConstants.RED_SCORING_SIDE;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;




public class HubArcDrive extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier strafeInput;
  private final Translation2d hubCenter;
  private final double scoringDistance;

  private final Rotation2d scoringSide;


  public HubArcDrive(SwerveSubsystem swerve, DoubleSupplier strafeInput, 
                     Translation2d hubCenter, double scoringDistance,
                     Rotation2d scoringSide) {
    this.swerve = swerve;
    this.strafeInput = strafeInput;
    this.hubCenter = hubCenter;
    this.scoringDistance = scoringDistance;
    
    this.scoringSide = scoringSide;
    
    
    addRequirements(swerve);
  }



  @Override
  public void initialize() {
    
  }

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
    
    //get desired distance vs real distance
    double distanceError = distanceFromHub - scoringDistance;
    double radialSpeed = -distanceError * 1.5; // p controller to keep distance
    radialSpeed = MathUtil.clamp(radialSpeed, -1.0, 1.0);//get speed
    
    if (Math.abs(distanceError) < 0.05) {
      radialSpeed = 0;
    }
    //movement around the arc
    double tangentialSpeed = strafeStrength * 2.0;
    
    // get x and y from distance to hub
    double radialX = Math.cos(angleFromHub);
    double radialY = Math.sin(angleFromHub);
    //get x and y for around the hub
    double tangentialX = -Math.sin(angleFromHub);
    double tangentialY = Math.cos(angleFromHub);
    
    double fieldVx = radialSpeed * radialX + tangentialSpeed * tangentialX;
    double fieldVy = radialSpeed * radialY + tangentialSpeed * tangentialY;
    //compare desired heading vs wanted heading
    Rotation2d desiredHeading = Rotation2d.fromRadians(angleFromHub + Math.PI);
    double headingError = desiredHeading.minus(currentHeading).getRadians();//get error
    headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));//wrap error
    
    // P controller
    double headingSpeed = headingError * 4.0; 
    
    // Add feedforward for smooth arc motion
    headingSpeed += tangentialSpeed / scoringDistance;
    
    // Gentle limit
    double maxHeadingSpeed = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();
    headingSpeed = MathUtil.clamp(headingSpeed, -maxHeadingSpeed * 0.5, maxHeadingSpeed * 0.5);
    
    //turn to robot oriented because the rotation and position is calculate around the robot
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldVx, fieldVy, headingSpeed, currentHeading
    );
    
    swerve.drive(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    Pose2d pose = swerve.getPose();
    if (scoringSide == RED_SCORING_SIDE && pose.getX() > 4.611) {
      System.out.print("Wrong side");
      return true;
    
    }
    else if (scoringSide == BLUE_SCORING_SIDE && pose.getX() < 11.901424) {
      System.out.print("Wrong side");
      return true;
    }
    return false;
  }
}