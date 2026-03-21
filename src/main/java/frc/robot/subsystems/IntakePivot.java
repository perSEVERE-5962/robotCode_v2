package frc.robot.subsystems;

import frc.robot.Constants;

public class IntakePivot extends Actuator {
  private static IntakePivot instance;

  private double targetPosition = 0;
  private static final double POSITION_TOLERANCE_ROTATIONS = 0.05;

  private IntakePivot() {
    super(
        Constants.CANDeviceIDs.kIntakePivotID,
        Constants.IntakePivotConstants.P,
        Constants.IntakePivotConstants.I,
        Constants.IntakePivotConstants.D,
        Constants.IntakePivotConstants.MinOutput,
        Constants.IntakePivotConstants.MaxOutput,
        Constants.IntakePivotConstants.FF,
        Constants.IntakePivotConstants.Iz,
        60f,
        11.5F,
        40,
        false,
        false,
        false,
        true);
  }

  @Override
  public void moveToPositionWithPID(double position) {
    targetPosition = position;
    super.moveToPositionWithPID(position);
  }

  public boolean isAtTarget() {
    return Math.abs(getPosition() - targetPosition) < POSITION_TOLERANCE_ROTATIONS;
  }

  public double getTemperature() {
    return getMotor().getMotorTemperature();
  }

  // Hardware accessors
  public double getTargetPosition() {
    return targetPosition;
  }

  public void setMotorPositionToScoring() {
    getMotor().getEncoder().setPosition(38.24);
  }

  public double getAppliedOutput() {
    return getMotor().getAppliedOutput();
  }

  public double getOutputCurrent() {
    return getMotor().getOutputCurrent();
  }

  public static IntakePivot getInstance() {
    if (instance == null) {
      instance = new IntakePivot();
    }
    return instance;
  }
}
