package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class MoveIntake extends Command {
  private Intake intake;
  private double speed;
  
  public MoveIntake(double speed) {
    intake = Intake.getInstance();
    this.speed = speed;

    addRequirements(intake);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    intake.move(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.move(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
