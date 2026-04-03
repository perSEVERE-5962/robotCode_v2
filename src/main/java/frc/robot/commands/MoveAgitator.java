package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator;
import java.util.function.BooleanSupplier;

public class MoveAgitator extends Command {
  private Agitator agitator;
  private double rpm;
  private BooleanSupplier arcDriveOn;

  public MoveAgitator() {

    //   agitator = Agitator.getInstance();
    //   addRequirements(agitator);
    // }

    // public MoveAgitator(double rpm) {
    //   agitator = Agitator.getInstance();
    //   this.rpm = rpm;
    //   addRequirements(agitator);
    // }

    // @Override
    // public void initialize() {}

    // @Override
    // public void execute() {
    //   agitator.runVelocity();
    //   }

    // @Override
    // public void end(boolean interrupted) {
    //   agitator.move(0);
    // }

    // @Override
    // public boolean isFinished() {
    //   return false;
    // }
  }
}
