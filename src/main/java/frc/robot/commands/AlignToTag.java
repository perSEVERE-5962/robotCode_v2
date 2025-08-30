// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.Cameras;
import frc.robot.subsystems.swervedrive.Vision.PoseObservation;
import swervelib.SwerveDrive;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTag extends Command {
  private Vision visionSubsystem;
  private SwerveSubsystem swerveSubsystem;
  private Cameras camera;
  private Cameras lastBestCamera = null;
  private int desiredTagId;

  /** Creates a new AlignToTag. */
  public AlignToTag(SwerveSubsystem swerve, int desiredTagId) {
    visionSubsystem = swerve.getVision();
    this.desiredTagId = desiredTagId;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Cameras bestCam = visionSubsystem.getbestCamera(desiredTagId); // call here
    if (bestCam != null) {
      lastBestCamera = bestCam; // update last seen camera
      System.out.println("Best camera: " + bestCam);}
   if (lastBestCamera != null) {
      System.out.println("Using last known camera: " + lastBestCamera);
  } else {
      System.out.println("No camera sees tag yet");
  }
  }

  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

