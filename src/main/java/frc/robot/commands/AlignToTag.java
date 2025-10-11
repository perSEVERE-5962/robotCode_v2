// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.Cameras;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlignToTag extends Command {
  private Vision visionSubsystem;
  private SwerveSubsystem swerveSubsystem;
private Cameras camera;
  private int desiredTag;
  private PhotonTrackedTarget target;
  private Transform2d offset;
//the hasTarget variablen could bed removed once our gyro is consistent, so then our alignment could align without even seeing the tag
  private boolean hasTarget = false;
  Pose2d targetPose;
 
private final PIDController xPID = new PIDController(10.0, 0.0, 2.0);
private final PIDController yPID = new PIDController(7.5, 0.0, 0.5);
private final PIDController rotationPID = new PIDController(10, 0.0, 1.0);

  private final double posTol = 0.07; //7 cm
  private final double angTol = Math.toRadians(2); // 2 degrees
 

  /** Creates a new AlignToTag. */
  public AlignToTag(SwerveSubsystem swerve, int desiredTag, Transform2d offset) {
    visionSubsystem = swerve.getVision();
    this.desiredTag = desiredTag;
    this.swerveSubsystem = swerve;
    this.offset = offset;



    //settling tolerance
    xPID.setTolerance(posTol);
    yPID.setTolerance(posTol);

    //wrapping and tolerance
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    rotationPID.setTolerance(angTol);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
        camera = visionSubsystem.getbestCamera(desiredTag);
        hasTarget = false;
        xPID.reset();
        yPID.reset();
        rotationPID.reset();

  }
  }

  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    camera = visionSubsystem.getbestCamera(desiredTag);
    if (camera == null) {
      swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
      hasTarget = false;
      SmartDashboard.putBoolean("AlignToReef/HasTarget", false);
      return;
  }
    
 Optional<PhotonPipelineResult> result = camera.getLatestResult();

    if (result.isEmpty() || !result.get().hasTargets()) {
      // no targets, stops the robot
      swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
      hasTarget = false;
      SmartDashboard.putBoolean("AlignToReef/HasTarget", false);
      return;
    }
    
    // finding the desired target
    target = null;
    for (PhotonTrackedTarget trackedTarget : result.get().getTargets()) {
      if (trackedTarget.getFiducialId() == desiredTag) {
        target = trackedTarget;
        break;
      }
    }

    if (target == null) {
      // Couldn't find the desired tag, stop the robot
      swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
      hasTarget = false;
      SmartDashboard.putBoolean("AlignToReef/HasTarget", false);
      return;
  }

  hasTarget = true;
  
    
      // calculate the targetpose relative to tag using the inputted offset
      

    Transform3d cameraToTarget = target.getBestCameraToTarget();
if (cameraToTarget == null) {
  swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  hasTarget = false;
  return;
}

// Calculate tag pose using 3D solver data + current robot pose
Pose2d currentRobotPose = swerveSubsystem.getPose();
Transform3d robotToCamera = camera.getRobotToCamera();
Transform3d robotToTarget = robotToCamera.plus(cameraToTarget);

Pose3d currentRobotPose3d = new Pose3d(
    currentRobotPose.getX(), 
    currentRobotPose.getY(), 
    0, 
    new edu.wpi.first.math.geometry.Rotation3d(0, 0, currentRobotPose.getRotation().getRadians())
);

Pose3d calculatedTagPose3d = currentRobotPose3d.plus(robotToTarget);
Pose2d calculatedTagPose2d = calculatedTagPose3d.toPose2d();
Pose2d calculatedTargetPose = calculatedTagPose2d.transformBy(offset);

if (calculatedTargetPose == null) {
  swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  hasTarget = false;
  return;
}
    targetPose = calculatedTargetPose;
        Pose2d currentPose = swerveSubsystem.getPose();

        // distance to targetpose
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double distance = Math.hypot(dx, dy);

        // speed scaling parameters
        double minSpeed = 0.1; // meters per second
        double maxSpeed = Constants.MAX_SPEED * 0.8; //use 80% of max speed for accuracy(maybe change later)
        double slowDistance = 1.0; // meters

        double speedFactor = MathUtil.clamp(distance / slowDistance, 0.0, 1.0);// clamp speed factor, changes when uner 1 meter
        double translationSpeed = minSpeed + (maxSpeed - minSpeed) * speedFactor;// final translation speed

        // PID control with scaling
        double xSpeed = MathUtil.clamp(xPID.calculate(currentPose.getX(), targetPose.getX()), -translationSpeed, translationSpeed);//clamp the calculate speed to the translation speed
        double ySpeed = MathUtil.clamp(yPID.calculate(currentPose.getY(), targetPose.getY()), -translationSpeed, translationSpeed);
        // limit rotation speed to 120 degrees per second
        double rotSpeed = MathUtil.clamp(rotationPID.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()), -Math.toRadians(120), Math.toRadians(120));

        swerveSubsystem.driveFieldOriented(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
    // SmartDashboard.putNumber("TargetPose/X", targetPose.getX());
    // SmartDashboard.putNumber("TargetPose/Y", targetPose.getY());
    // SmartDashboard.putNumber("TargetPose/Rotation", targetPose.getRotation().getDegrees());

    // SmartDashboard.putNumber("RobotPose/X", currentPose.getX());
    // SmartDashboard.putNumber("RobotPose/Y", currentPose.getY());
    // SmartDashboard.putNumber("RobotPose/Rotation", currentPose.getRotation().getDegrees());

}


  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!hasTarget || targetPose == null) {
      return false;
  }
  Pose2d currentPose = swerveSubsystem.getPose();
  Rotation2d currentHeading = swerveSubsystem.getHeading();
  
    // calculate errors
    double xError = Math.abs(targetPose.getX() - currentPose.getX());
    double yError = Math.abs(targetPose.getY() - currentPose.getY());
    double rotationError = Math.abs(
    targetPose.getRotation().getRadians() - currentHeading.getRadians()
    );
    
    // normalizing rotation error
    while (rotationError > Math.PI) rotationError -= 2 * Math.PI;
    rotationError = Math.abs(rotationError);
    
    // heck if within tolerances
    boolean positionOK = xError <= posTol && yError <= posTol;
    boolean rotationOK = rotationError <= angTol;
    
    return positionOK && rotationOK;
}
}






