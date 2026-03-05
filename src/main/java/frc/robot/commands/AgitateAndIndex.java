// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Indexer;

/* Agitate and index while HubArcDrive. */
public class AgitateAndIndex extends Command {
  // Creates a new AgitateAndIndex. */
  private double agitateSpeed;
  private Agitator agitator;
  private BooleanSupplier arcDriveOn;
  private Indexer indexer;
  private double indexRPM;
  public AgitateAndIndex(double agitateSpeed, double indexRPM, BooleanSupplier arcDriveOn) {
    this.agitateSpeed = agitateSpeed;
    agitator = Agitator.getInstance();
    this.arcDriveOn= arcDriveOn;
    this.indexRPM = indexRPM;
    indexer = Indexer.getInstance();
    addRequirements(agitator);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(arcDriveOn.getAsBoolean()&&HubArcDrive.checkHeadingError()){
    indexer.moveToVelocityWithPID(indexRPM);
    agitator.move(agitateSpeed);
    }
    else if(arcDriveOn.getAsBoolean()&&!HubArcDrive.checkHeadingError()){
      indexer.moveToVelocityWithPID(0);
      agitator.move(0);
    }
    else{
    indexer.moveToVelocityWithPID(indexRPM);
    agitator.move(agitateSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.move(0);
    agitator.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
