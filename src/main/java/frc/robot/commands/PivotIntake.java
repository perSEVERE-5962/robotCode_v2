// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotIntake extends Command {
  /** Creates a new PivotIntake. */
  private IntakePivot intakePivot;

  private double rpm;

  public PivotIntake(double rpm) {
    intakePivot = IntakePivot.getInstance();
    this.rpm = rpm;
    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {
        intakePivot.move(rpm);

  }

  @Override
  public void execute() {
    // System.out.println(intakePivot.getPosition());
  }

  @Override
  public void end(boolean interrupted) {
    intakePivot.move(0);
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
