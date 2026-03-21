package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoller;

public class MoveIntake extends Command {
  private IntakeRoller intakeRoller;
  private double rpm;

  public MoveIntake(double rpm) {
    intakeRoller = IntakeRoller.getInstance();
    this.rpm = rpm;

    addRequirements(intakeRoller);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeRoller.moveToVelocityWithPID(rpm);
  }

  @Override
  public void end(boolean interrupted) {
    intakeRoller.move(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
