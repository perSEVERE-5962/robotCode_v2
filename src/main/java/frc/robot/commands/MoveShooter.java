package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class MoveShooter extends Command {
  private Shooter shooter;
  private double rpm;
  
  public MoveShooter(double rpm) {
    shooter = Shooter.getInstance();
    this.rpm = rpm;
    
    addRequirements(shooter);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    shooter.moveToVelocityWithPID(rpm);
    //System.out.println(shooter.getMotorVelocity());
  }

  @Override
  public void end(boolean interrupted) {
    shooter.move(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
