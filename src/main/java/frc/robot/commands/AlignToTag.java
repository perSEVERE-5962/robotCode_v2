// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.Cameras;
import frc.robot.subsystems.swervedrive.Vision.PoseObservation;
import swervelib.SwerveDrive;
import frc.robot.Constants.AlignConstants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTag extends Command {
  private Vision visionSubsystem;
  private SwerveSubsystem swerveSubsystem;
  private Cameras lastBestCamera = null;
  private int desiredTagId;
  private SwerveDrive swerveDrive;
  private Pose2d pose;
  private HolonomicDriveController holonomicDriveController =
    new HolonomicDriveController(
        new PIDController(AlignConstants.kPID_XKP, AlignConstants.kPID_XKI, AlignConstants.kPID_XKD),
        new PIDController(AlignConstants.kPID_YKP, AlignConstants.kPID_YKI, AlignConstants.kPID_YKD),
        new ProfiledPIDController(AlignConstants.kPID_TKP,AlignConstants.kPID_TKI, AlignConstants.kPID_TKD,
        new TrapezoidProfile.Constraints(10, 10 )));


  /** Creates a new AlignToTag. */
  public AlignToTag(SwerveSubsystem swerve, int desiredTagId, Supplier<Pose2d> pose) {
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
  final Transform3d robotToCamera = bestCam.getRobotToCamera();
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

