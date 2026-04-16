package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.TunableNumber;

// driver points the robot, copilot holds the button, balls come out.
// constant RPM, no aiming, no zone logic. adjustable from dashboard.
public class FeedEject extends Command {
  private static final TunableNumber feedRPM = new TunableNumber("Feed/RPM", 4000);

  private final Shooter shooter;
  private final Indexer indexer;
  private final Agitator agitator;

  public FeedEject() {
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();
    agitator = Agitator.getInstance();
    addRequirements(shooter, indexer, agitator);
  }

  @Override
  public void execute() {
    shooter.moveToVelocityWithPID(feedRPM.get());

    if (Math.abs(4000 - shooter.getVelocity()) < Shooter.getToleranceRPM()) {
      indexer.moveToVelocityWithPID(Indexer.getTunableTargetSpeed());
      agitator.moveToVelocityWithPID(Agitator.getTunableTargetRPM());
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.move(0);
    indexer.move(0);
    agitator.move(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
