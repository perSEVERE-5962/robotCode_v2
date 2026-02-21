// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeActuator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotIntake extends Command {
  /** Creates a new PivotIntake. */
  private IntakeActuator intakeActuator;
  private double speed;
  public PivotIntake(double speed) {
   intakeActuator = IntakeActuator.getInstance();
    this.speed=speed;
    addRequirements(intakeActuator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeActuator.move(speed);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    
  return false;
  }
}

