package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hanger;

public class DeployHanger extends Command {
  private Hanger hanger;

  public DeployHanger() {
    hanger = Hanger.getInstance();

    addRequirements(hanger);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    hanger.moveToPositionWithPID(Constants.MotorConstants.UP_HANGER_POS);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
