package frc.robot.commands;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.SystemMenuBar;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator;

public class MoveAgitator extends Command {
  private Agitator agitator;
  private double rpm;

  public MoveAgitator(double rpm) {
    agitator = Agitator.getInstance();
    this.rpm = rpm;
    addRequirements(agitator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
      agitator.moveToVelocityWithPID(rpm);
      System.out.println(agitator.getMotorVelocity());
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
