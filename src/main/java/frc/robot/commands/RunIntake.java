package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RunIntake extends SequentialCommandGroup {
  public RunIntake() {
    super(new DeployIntake(), new MoveIntake());
  }
}
