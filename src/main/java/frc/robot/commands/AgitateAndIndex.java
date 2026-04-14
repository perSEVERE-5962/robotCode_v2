package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Indexer;

public class AgitateAndIndex extends Command {
  private double agitateRPM;
  private Agitator agitator;
  private Indexer indexer;
  private double indexRPM;

  public AgitateAndIndex(double indexRPM, double agitateRPM) {
    this.agitateRPM = agitateRPM;
    agitator = Agitator.getInstance();

    this.indexRPM = indexRPM;
    indexer = Indexer.getInstance();
    addRequirements(agitator, indexer);
  }

  @Override
  public void initialize() {
    agitator.moveToVelocityWithPID(agitateRPM);
  }

  @Override
  public void execute() {

    indexer.moveToVelocityWithPID(indexRPM);
    agitator.moveToVelocityWithPID(agitateRPM);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.move(0);
    agitator.move(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
