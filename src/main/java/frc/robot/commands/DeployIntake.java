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
    double currentPos = intakePivot.getPosition();
    if (currentPos >= (Constants.MotorConstants.OUT_INTAKE_POS - .08)
        && currentPos <= (Constants.MotorConstants.OUT_INTAKE_POS + .08)) {
      return true;
    } else {
      return false;
    }
  }
}
