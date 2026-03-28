package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Indexer;
import java.util.function.BooleanSupplier;

public class AgitateAndIndex extends Command {
  private double agitateRPM;
  private Agitator agitator;
  private BooleanSupplier arcDriveOn;
  private Indexer indexer;
  private double indexRPM;

  public AgitateAndIndex(double agitateRPM, double indexRPM) {
    this.agitateRPM = agitateRPM;
    agitator = Agitator.getInstance();
    this.arcDriveOn = arcDriveOn;
    this.indexRPM = indexRPM;
    indexer = Indexer.getInstance();
    addRequirements(agitator, indexer);
  }

  @Override
  public void initialize() {
    agitator.moveToVelocityWithPID(-600);
  }

  @Override
  public void execute() {
   
      indexer.moveToVelocityWithPID(indexer.getTunableTargetSpeed());
      agitator.moveToVelocityWithPID(agitateRPM);
    
   
  }

  @Override
  public void end(boolean interrupted) {
    indexer.moveToVelocityWithPID(0);
    agitator.moveToVelocityWithPID(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
