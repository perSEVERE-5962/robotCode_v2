// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.HubScoringUtil;

public class DriveToHub extends Command {

  private final SwerveSubsystem swerve;
  private final Translation2d hubCenter;
  private final double scoringDistance;
  private final Rotation2d scoringSide;
  private final double arcWidthDegrees;
  private Command driveCommand;

  public DriveToHub(
      SwerveSubsystem swerve,
      Translation2d hubCenter,
      double scoringDistance,
      Rotation2d scoringSide,
      double arcWidthDegrees) {
    this.swerve = swerve;
    this.hubCenter = hubCenter;
    this.scoringDistance = scoringDistance;
    this.scoringSide = scoringSide;
    this.arcWidthDegrees = arcWidthDegrees;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // get closest point on the arc
    Translation2d robotPosition = swerve.getPose().getTranslation();
    Pose2d targetPose =
        HubScoringUtil.getClosestScoringPose(
            robotPosition, hubCenter, scoringDistance, scoringSide, arcWidthDegrees);

    // make command using pathplanner to drive to pose
    driveCommand = swerve.driveToPose(targetPose);
    CommandScheduler.getInstance().schedule(driveCommand);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (driveCommand != null && driveCommand.isScheduled()) {
      driveCommand.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    // Finish when the drive command completes
    return driveCommand != null && !driveCommand.isScheduled();
  }
}
