// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.telemetry.AgitatorTelemetry;

/*
 * Command to speed up shooter then run indexer.
 */
public class SpeedUpThenIndex extends Command {
  /** Creates a new SpeedUpThenIndex. */
  private Shooter shooter;

  private Agitator agitator;
  private Indexer indexer;
  private AgitatorTelemetry agitatorTelemetry;

  public SpeedUpThenIndex() {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();
    agitator = Agitator.getInstance();
    agitatorTelemetry = new AgitatorTelemetry();
    addRequirements(shooter, indexer, agitator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.moveToVelocityWithPID(shooter.getTunableTargetRPM());
    agitator.moveToVelocityWithPID(-5000);
    indexer.moveToVelocityWithPID(-5000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isAtSpeed()) {
      indexer.moveToVelocityWithPID(indexer.getTunableTargetSpeed());
      // System.out.println(indexer.getMotorVelocity());
      agitator.moveToVelocityWithPID(5990);
    }
    // if(AgitatorTelemetry.isStalled()){
    //         agitator.moveToVelocityWithPID(-2000);

    // }
    // System.out.println("one");
    // System.out.println(shooter.getTunableTargetRPM());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.moveToVelocityWithPID(0);

    indexer.moveToVelocityWithPID(0);
    // System.out.println();
    agitator.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
