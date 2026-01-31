// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpeedUpThenIndex extends Command {
  /** Creates a new SpeedUpThenIndex. */
  private Shooter shooter;
  private Indexer indexer;
  public SpeedUpThenIndex() {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();

    addRequirements(shooter, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CommandScheduler.getInstance().schedule(new MoveShooter(Constants.MotorConstants.DESIRED_SHOOTER_SPEED));
    final Command waitUntilSpeed = Commands.waitUntil(() -> shooter.getMotorVelocity() == Constants.MotorConstants.DESIRED_SHOOTER_SPEED);
    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(waitUntilSpeed, new MoveIndexer(Constants.MotorConstants.DESIRED_INDEXER_SPEED)));
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
