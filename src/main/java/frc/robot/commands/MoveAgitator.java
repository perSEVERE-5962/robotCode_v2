package frc.robot.commands;

import java.util.function.BooleanSupplier;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.SystemMenuBar;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator;

public class MoveAgitator extends Command {
  private Agitator agitator;
  private double rpm;
  private BooleanSupplier arcDriveOn;

  public MoveAgitator(double rpm, BooleanSupplier arcDriveOn) {
    this.rpm = rpm;
    agitator = Agitator.getInstance();
    this.arcDriveOn = arcDriveOn;
    addRequirements(agitator);
  }

  public MoveAgitator(double rpm) {
    agitator = Agitator.getInstance();
    this.rpm = rpm;
    arcDriveOn = () -> false;

    addRequirements(agitator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (arcDriveOn != null && arcDriveOn.getAsBoolean() && !HubArcDrive.checkHeadingError()) {
      agitator.moveToVelocityWithPID(0);
    } else {
      agitator.moveToVelocityWithPID(rpm);
      System.out.println(agitator.getMotorVelocity());
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
