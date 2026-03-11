package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoller;

public class MoveIntake extends Command {
  private IntakeRoller intake;
  
  public MoveIntake() {
    intake = IntakeRoller.getInstance();

    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.move(intake.getTunableSpeed());
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
