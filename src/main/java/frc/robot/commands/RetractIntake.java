package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
    intakeActuator.moveToPositionWithPID(Constants.MotorConstants.IN_INTAKE_POS);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (Math.abs(Constants.MotorConstants.IN_INTAKE_POS - intakeActuator.getPosition()) < Constants.MotorConstants.INTAKE_TOLERANCE) {
      return true;
    }
    return false;
  }
}
