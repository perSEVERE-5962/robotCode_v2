package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeActuator;

public class RetractIntake extends Command {
  private IntakeActuator intakeActuator;

  public RetractIntake() {
    intakeActuator = IntakeActuator.getInstance();

    addRequirements(intakeActuator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeActuator.moveToPositionWithPID(intakeActuator.getTunableRetractPosition());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
