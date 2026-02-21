package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Indexer;
import frc.robot.telemetry.IndexerTelemetry;
import frc.robot.Constants;

public class MoveAgitator extends Command {
  private Agitator agitator;
  private double speed;
  private BooleanSupplier arcDriveOn;
    //private IndexerTelemetry indextelem = new IndexerTelemetry();
public MoveAgitator(double speed, BooleanSupplier arcDriveOn) {
    this.speed = speed;
    agitator = Agitator.getInstance();
    this.arcDriveOn= arcDriveOn;
    addRequirements(agitator);
  }
  public MoveAgitator(double speed) {
    agitator = Agitator.getInstance();
    this.speed=speed;
    addRequirements(agitator);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    //agitator.move(speed);
    // if (indextelem.isJamDetected()) {
    //     agitator.move(-Constants.MotorConstants.DESIRED_AGITATOR_SPEED);
    // } else {
    //     agitator.move(speed); 
    // }
    //agitator.move(speed); 
    if(arcDriveOn.getAsBoolean()&&HubArcDrive.checkHeadingError()){
    agitator.move(speed);
    }
    else if(arcDriveOn.getAsBoolean()&&!HubArcDrive.checkHeadingError()){
      agitator.move(0);
    }
    else{
    agitator.move(speed);
    }
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
