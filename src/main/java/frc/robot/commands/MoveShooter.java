package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.util.ShotCalculator;


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
    double override = ShotCalculator.getInstance().getRpmOverride();
    double targetRPM;
    if (override > 0){
      targetRPM = override;
    }else{
      targetRPM = rpm;
    }
    shooter.moveToVelocityWithPID(targetRPM);
  }

  @Override
  public void execute() {
    // shooter.moveToVelocityWithPID(rpm);
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
