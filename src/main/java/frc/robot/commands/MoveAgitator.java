package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Agitator;

public class MoveAgitator extends Command {
  private Agitator agitator;

  public MoveAgitator() {
    agitator = Agitator.getInstance();

    addRequirements(agitator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    agitator.move(Constants.MotorConstants.DESIRED_AGITATOR_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    agitator.move(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
