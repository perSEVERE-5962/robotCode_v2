// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Agitator;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpeedUpThenIndex extends Command {
  /** Creates a new SpeedUpThenIndex. */
  private Shooter shooter;
  private Indexer indexer;
  private Agitator agitator;
  public SpeedUpThenIndex() {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();
    agitator = Agitator.getInstance();

    addRequirements(shooter, indexer, agitator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.moveToVelocityWithPID(Constants.MotorConstants.DESIRED_SHOOTER_RPM);
    if (Math.abs(Constants.MotorConstants.DESIRED_SHOOTER_RPM - shooter.getVelocity()) < Constants.MotorConstants.SHOOTER_TOLERANCE) {
      indexer.moveToVelocityWithPID(Constants.MotorConstants.DESIRED_INDEXER_RPM);
      agitator.move(Constants.MotorConstants.DESIRED_AGITATOR_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.move(0);
    indexer.move(0);
    agitator.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
