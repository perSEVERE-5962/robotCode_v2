// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.TunableNumber;

/**
 * Spins up the shooter, then feeds balls once at speed. Uses a firing latch so
 * the indexer keeps running through brief RPM dips from ball impacts. Only stops
 * feeding if RPM drops below an under-shooting threshold for a sustained period.
 */
public class SpeedUpThenIndex extends Command {
  private Shooter shooter;
  private Agitator agitator;
  private Indexer indexer;

  // Firing hysteresis state
  private boolean reachedSpeed = false;
  private boolean feeding = false;
  private double underShootingSince = -1;

  // Tunable thresholds for the under-shooting exit
  private static final TunableNumber underShootPct =
      new TunableNumber("Shooter/Firing/UnderShootPct", 0.85);
  private static final TunableNumber underShootDebounceMs =
      new TunableNumber("Shooter/Firing/DebounceMs", 400);

  public SpeedUpThenIndex() {
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();
    agitator = Agitator.getInstance();
    addRequirements(shooter, indexer, agitator);
  }

  @Override
  public void initialize() {
    reachedSpeed = false;
    feeding = false;
    underShootingSince = -1;
    shooter.moveToVelocityWithPID(shooter.getTunableTargetRPM());
    agitator.moveToVelocityWithPID(-500);
  }

  @Override
  public void execute() {
    boolean atSpeed = shooter.isAtSpeed();

    // Latch: once we reach speed, start feeding
    if (atSpeed && !reachedSpeed) {
      reachedSpeed = true;
      feeding = true;
    }

    // Under-shooting check: only matters while we're in the firing latch
    if (reachedSpeed) {
      double velocity = shooter.getVelocityRPM();
      double target = shooter.getTargetRPM();
      double threshold = target * underShootPct.get();

      if (velocity < threshold) {
        double now = Timer.getFPGATimestamp();
        if (underShootingSince < 0) {
          underShootingSince = now;
        } else if ((now - underShootingSince) * 1000.0 > underShootDebounceMs.get()) {
          // Sustained under-shooting, stop feeding until RPM recovers
          feeding = false;
        }
      } else {
        underShootingSince = -1;
        // RPM recovered above threshold, resume feeding if we'd stopped
        if (!feeding) {
          feeding = true;
        }
      }
    }

    if (feeding) {
      indexer.moveToVelocityWithPID(indexer.getTunableTargetSpeed());
      agitator.moveToVelocityWithPID(agitator.getTunableTargetRPM());
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.moveToVelocityWithPID(0);
    indexer.moveToVelocityWithPID(0);
    agitator.moveToVelocityWithPID(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
