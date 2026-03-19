package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;

public class RetractIntake extends Command {
  private IntakePivot intakePivot;

  public RetractIntake() {
    intakePivot = IntakePivot.getInstance();

    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakePivot.moveToPositionWithPID(Constants.MotorConstants.IN_INTAKE_POS);
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    double currentPos = intakePivot.getPosition();
    if (currentPos>=(Constants.MotorConstants.IN_INTAKE_POS-.2)&&currentPos<=(Constants.MotorConstants.IN_INTAKE_POS+.2)){
      return true;
     }
    else{
      return false;
  }
  }
}
