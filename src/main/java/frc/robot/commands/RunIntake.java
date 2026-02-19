package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class RunIntake extends SequentialCommandGroup {
  public RunIntake() {
    super(new DeployIntake(), new MoveIntake(Constants.MotorConstants.DESIRED_INTAKE_SPEED));
  }
}
