package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoller;

public class MoveIntake extends Command {
  private IntakeRoller intakeRoller;

  public MoveIntake() {
    intakeRoller = IntakeRoller.getInstance();

    addRequirements(intakeRoller);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeRoller.moveToVelocityWithPID(intakeRoller.getTunableSpeed());
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
