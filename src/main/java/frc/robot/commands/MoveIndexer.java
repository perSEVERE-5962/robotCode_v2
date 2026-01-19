package frc.robot.commands;

import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveIndexer extends Command {
  private Indexer indexer;
  private double speed;
  
  public MoveIndexer(double speed) {
    this.speed = speed;
    indexer = Indexer.getInstance();

    addRequirements(indexer);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    indexer.move(speed);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.move(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
