// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRoller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoldAndIntake extends Command {
  /** Creates a new HoldPivotWhileIntaking. */
  private IntakeRoller intakeRoller;

  private IntakePivot intakePivot;

  public HoldAndIntake() {
    intakeRoller = IntakeRoller.getInstance();
    intakePivot = IntakePivot.getInstance();

    addRequirements(intakeRoller, intakePivot);
  }

  @Override
  public void initialize() {
    intakePivot.moveToPositionWithPID(Constants.MotorConstants.OUT_INTAKE_POS);
  }

  @Override
  public void execute() {
    intakeRoller.moveToVelocityWithPID(intakeRoller.getTunableSpeed());
  }

  @Override
  public void end(boolean interrupted) {
    intakeRoller.move(0);
    intakePivot.move(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
