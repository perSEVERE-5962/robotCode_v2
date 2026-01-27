package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeActuator;

public class DeployIntake extends Command {
  private IntakeActuator intakeActuator;

  public DeployIntake() {
    intakeActuator = IntakeActuator.getInstance();

    addRequirements(intakeActuator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeActuator.moveToPositionWithPID(Constants.MotorConstants.OUT_INTAKE_POS);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
