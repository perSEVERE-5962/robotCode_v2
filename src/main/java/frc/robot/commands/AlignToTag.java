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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.List;


public class AlignToTag extends Command {
  private Vision visionSubsystem;
  private SwerveSubsystem swerveSubsystem;
  private Cameras camera;
  private int desiredTag;
  private PhotonTrackedTarget target;
  private Transform2d offset;
  //consider not stopping the robot when hastarget becomes false, allowing the robot to align using gyro.
  private boolean hasTarget = false;
  Pose2d targetPose;
 
  private final ProfiledPIDController xPID =
      new ProfiledPIDController(
          4.0, 0.0, 0.0,
          new TrapezoidProfile.Constraints(
              3.0,   // max velocity(for tuning)
              3.0    // max accelleration(for tuning)
          )
      );

  private final ProfiledPIDController yPID =
      new ProfiledPIDController(
          4.0, 0.0, 0.0,
          new TrapezoidProfile.Constraints(
              3.0,
              3.0
          )
      );

  private final ProfiledPIDController rotationPID =
      new ProfiledPIDController(
          4.0, 0.0, 0.0,
          new TrapezoidProfile.Constraints(
              Math.toRadians(180), 
              Math.toRadians(360)  
          )
      );
  private final double posTol = 0.07; //7 cm
  private final double angTol = Math.toRadians(2); // 2 degrees
 

  /** Creates a new AlignToTag. */
  public AlignToTag(SwerveSubsystem swerve, int desiredTag, Transform2d offset) {
    visionSubsystem = swerve.getVision();
    this.desiredTag = desiredTag;
    this.swerveSubsystem = swerve;
    this.offset = offset;


    addRequirements(swerveSubsystem);
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

    Pose2d pose = swerveSubsystem.getPose();
    xPID.reset(pose.getX());
    yPID.reset(pose.getY());
    rotationPID.reset(pose.getRotation().getRadians());

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
    
    List<PhotonPipelineResult> results = camera.resultsList;
    target = null;

    for (int i = 0; i < results.size(); i++) {
      PhotonPipelineResult result = results.get(i);
      if (result.hasTargets()) {
        for (int j = 0; i < result.getTargets().size(); j++) {
          PhotonTrackedTarget trackedTarget = result.getTargets().get(i); {
              if (trackedTarget.getFiducialId() == desiredTag) {
                  target = trackedTarget; 
                  break;
              }
          }
          if (target != null) {
            break;
          }
        }
      }
    }
  


    if (target == null) {
      // Couldn't find the desired tag, stop the robot
      swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
      hasTarget = false;
      SmartDashboard.putBoolean("AlignToReef/HasTarget", false);
      return;
    }

  
    
    Transform3d cameraToTarget = target.getBestCameraToTarget();
    if (cameraToTarget == null) {
      swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
      hasTarget = false;
      return;
    }

    hasTarget = true;


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


    targetPose = calculatedTargetPose;
    Pose2d currentPose = swerveSubsystem.getPose();

        
    double xSpeed = xPID.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yPID.calculate(currentPose.getY(), targetPose.getY());
        
    double rotSpeed = rotationPID.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    swerveSubsystem.driveFieldOriented(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
   

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasTarget
      && xPID.atGoal()
      && yPID.atGoal()
      && rotationPID.atGoal();
  }
}






