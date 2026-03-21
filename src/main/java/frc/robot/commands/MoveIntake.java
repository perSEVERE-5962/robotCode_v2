package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoller;

public class MoveIntake extends Command {
  private Intake intake;
  private double speed;

  public MoveIntake(double speed) {
    intake = Intake.getInstance();
    this.speed = speed;

    addRequirements(intakeRoller);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeRoller.moveToVelocityWithPID(intakeRoller.getTunableSpeed());
  }

  @Override
  public void end(boolean interrupted) {
    intakeRoller.move(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
