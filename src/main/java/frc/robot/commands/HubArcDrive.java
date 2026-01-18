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
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Command to drive left/right along the scoring arc around the hub.
 * The robot maintains a constant distance from the hub and always faces it.
 * Forward/backward input is disabled - only strafe (left/right) works.
 */





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
    // Nothing needed
  }

  @Override
   public void execute() {
    // Get joystick input
    double strafe = strafeInput.getAsDouble();
    
    // Get current robot state
    Translation2d robotPos = swerve.getPose().getTranslation();
    Rotation2d currentHeading = swerve.getHeading();
    
    // Calculate position relative to hub
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
    if (atLeftLimit && strafe > 0) {
      strafe = 0;
    }
    if (atRightLimit && strafe < 0) {
      strafe = 0;
    }
    
    // === SIMPLE DISTANCE CONTROL ===
    double distanceError = distanceFromHub - scoringDistance;
    double radialSpeed = -distanceError * 1.5; // Simple, constant gain
    radialSpeed = MathUtil.clamp(radialSpeed, -1.0, 1.0);
    
    if (Math.abs(distanceError) < 0.05) {
      radialSpeed = 0;
    }
    
    // === TANGENTIAL MOVEMENT ===
    double tangentialSpeed = strafe * 2.0;
    
    // === CONVERT TO FIELD COORDINATES ===
    double radialX = Math.cos(angleFromHub);
    double radialY = Math.sin(angleFromHub);
    double tangentialX = -Math.sin(angleFromHub);
    double tangentialY = Math.cos(angleFromHub);
    
    double fieldVx = radialSpeed * radialX + tangentialSpeed * tangentialX;
    double fieldVy = radialSpeed * radialY + tangentialSpeed * tangentialY;
    
    // === SIMPLE HEADING CONTROL ===
    Rotation2d desiredHeading = Rotation2d.fromRadians(angleFromHub + Math.PI);
    double headingError = desiredHeading.minus(currentHeading).getRadians();
    headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
    
    // Simple P controller with feedforward
    double omega = headingError * 4.0; // Constant, gentle gain
    
    // Add feedforward for smooth arc motion
    omega += tangentialSpeed / scoringDistance;
    
    // Gentle limit
    double maxOmega = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();
    omega = MathUtil.clamp(omega, -maxOmega * 0.5, maxOmega * 0.5);
    
    // === DRIVE ===
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldVx, fieldVy, omega, currentHeading
    );
    
    swerve.drive(speeds);
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