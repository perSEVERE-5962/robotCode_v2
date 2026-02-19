package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator;

public class MoveAgitator extends Command {
  private Agitator agitator;
  private double speed;
  
  public MoveAgitator(double speed) {
    agitator = Agitator.getInstance();
    this.speed = speed;

    addRequirements(agitator);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    agitator.move(speed);
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
