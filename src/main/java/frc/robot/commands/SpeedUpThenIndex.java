// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

/*
 * Command to speed up shooter then run indexer.
 */
public class SpeedUpThenIndex extends Command {
  /** Creates a new SpeedUpThenIndex. */
  private Shooter shooter;
  private Agitator agitator;
  private Indexer indexer;

  public SpeedUpThenIndex() {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();
    agitator = Agitator.getInstance();
    addRequirements(shooter, indexer,agitator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("one");
    shooter.moveToVelocityWithPID(shooter.getTunableTargetRPM());
    if (shooter.isAtSpeed()) {
      indexer.moveToVelocityWithPID(indexer.getTunableTargetSpeed());
      agitator.move(.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().schedule(new MoveShooter(0));
    agitator.move(0);
    CommandScheduler.getInstance()
        .schedule(new MoveIndexer(-Constants.MotorConstants.DESIRED_INDEXER_RPM));
    final Command waitTime = Commands.waitSeconds(0.25);
    CommandScheduler.getInstance()
        .schedule(new SequentialCommandGroup(waitTime, new MoveIndexer(0)));
  }      

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
