package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;

public class RetractIntake extends Command {
  private IntakePivot intakePivot;

  public RetractIntake() {
    intakePivot = IntakePivot.getInstance();

    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {
    intakePivot.moveToPositionWithPID(Constants.MotorConstants.IN_INTAKE_POS);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    intakePivot.move(0);
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(Constants.MotorConstants.IN_INTAKE_POS - intakePivot.getPosition())
        < Constants.MotorConstants.INTAKE_POS_TOLERANCE) {
      return true;
    }
    return false;
  }
}
