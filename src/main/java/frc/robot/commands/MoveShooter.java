package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class MoveShooter extends Command {
  private Shooter shooter;
  
  
  public MoveShooter(double speed) {
    shooter = Shooter.getInstance();
    

    addRequirements(shooter);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    shooter.runShooter(1.0);
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
