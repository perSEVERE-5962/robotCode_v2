package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;

public class DeployIntake extends Command {
  private IntakePivot intakePivot;

  public DeployIntake() {
    intakePivot = IntakePivot.getInstance();

    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {
    intakePivot.moveToPositionWithPID(Constants.MotorConstants.OUT_INTAKE_POS);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (Math.abs(Constants.MotorConstants.OUT_INTAKE_POS - intakePivot.getPosition())
        < Constants.MotorConstants.INTAKE_POS_TOLERANCE) {
      return true;
    }
    return false;
  }
}
