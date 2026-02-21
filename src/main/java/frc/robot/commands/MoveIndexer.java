package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import java.util.function.BooleanSupplier;
public class MoveIndexer extends Command {
  private Indexer indexer;
  private double rpm;
  private BooleanSupplier arcDriveOn;
  public MoveIndexer(double rpm, BooleanSupplier arcDriveOn) {
    this.rpm = rpm;
    indexer = Indexer.getInstance();
    this.arcDriveOn= arcDriveOn;
    addRequirements(indexer);
  }
  public MoveIndexer(double rpm){
    this.rpm = rpm;
    indexer = Indexer.getInstance();
    this.arcDriveOn = () -> false;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(arcDriveOn.getAsBoolean()&&HubArcDrive.checkHeadingError()){
    indexer.moveToVelocityWithPID(rpm);
    }
    else if(arcDriveOn.getAsBoolean()&&!HubArcDrive.checkHeadingError()){
      indexer.moveToVelocityWithPID(0);
    }
    else{
    indexer.moveToVelocityWithPID(rpm);
    }
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
