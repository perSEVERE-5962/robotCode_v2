package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class MoveIndexer extends Command {
  private Indexer indexer;
  private double rpm;
  private boolean arcdriveOn;
  public MoveIndexer(double rpm, boolean arcDriveOn) {
    this.rpm = rpm;
    indexer = Indexer.getInstance();
    this.arcdriveOn= arcDriveOn;
    addRequirements(indexer);
  }
  public MoveIndexer(double rpm){
    this.rpm = rpm;
    indexer = Indexer.getInstance();
    arcdriveOn=false;
    addRequirements(indexer);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(arcdriveOn&&HubArcDrive.checkHeadingError()){
    indexer.moveToVelocityWithPID(rpm);
    }
    else if(arcdriveOn&&!HubArcDrive.checkHeadingError()){
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
