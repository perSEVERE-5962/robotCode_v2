package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Indexer;

public class AgitateAndIndex extends Command {
  private double agitateRPM;
  private Agitator agitator;
  private BooleanSupplier arcDriveOn;
  private Indexer indexer;
  private double indexRPM;

  public AgitateAndIndex(double agitateRPM, double indexRPM, BooleanSupplier arcDriveOn) {
    this.agitateRPM = agitateRPM;
    agitator = Agitator.getInstance();
    this.arcDriveOn = arcDriveOn;
    this.indexRPM = indexRPM;
    indexer = Indexer.getInstance();
    addRequirements(agitator, indexer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (arcDriveOn.getAsBoolean() && HubArcDrive.checkHeadingError()) {
      indexer.moveToVelocityWithPID(indexRPM);
      agitator.moveToVelocityWithPID(agitateRPM);
    } else if (arcDriveOn.getAsBoolean() && !HubArcDrive.checkHeadingError()) {
      indexer.moveToVelocityWithPID(0);
      agitator.moveToVelocityWithPID(0);
    } else {
      indexer.moveToVelocityWithPID(indexRPM);
      agitator.moveToVelocityWithPID(agitateRPM);
    }
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
