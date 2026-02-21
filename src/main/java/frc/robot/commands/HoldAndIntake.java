// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeActuator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoldAndIntake extends Command {
  /** Creates a new HoldPivotWhileIntaking. */
  private Intake intake;
  private IntakeActuator intakeActuator;

  public HoldAndIntake() {
    intake = Intake.getInstance();
    intakeActuator = IntakeActuator.getInstance();

    addRequirements(intake,intakeActuator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.move(intake.getTunableSpeed());
    intakeActuator.moveToPositionWithPID(Constants.MotorConstants.OUT_INTAKE_POS);

  }

  @Override
  public void end(boolean interrupted) {
    intake.move(0);
    intakeActuator.move(0);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}


