package frc.robot.subsystems;

import frc.robot.Constants;

public class IntakePivot extends Actuator {
  private static IntakePivot instance;

  private double targetPosition = 0;

  private IntakePivot() {
    super(
        Constants.CANDeviceIDs.kIntakePivotID,
        Constants.IntakePivotConstants.kP,
        Constants.IntakePivotConstants.kI,
        Constants.IntakePivotConstants.kD,
        Constants.IntakePivotConstants.kMinOutput,
        Constants.IntakePivotConstants.kMaxOutput,
        Constants.IntakePivotConstants.kS,
        Constants.IntakePivotConstants.kV,
        0,
        Constants.IntakePivotConstants.kCos,
        Constants.IntakePivotConstants.kCosRatio,
        Constants.IntakePivotConstants.kIz,
        60f,
        11.5f,
        40,
        false,
        false,
        false,
        true,
        false);
  }

  @Override
  public void moveToPositionWithPID(double position) {
    targetPosition = position;
    super.moveToPositionWithPID(position);
  }

  public boolean isAtTarget() {
    return Math.abs(getPosition() - targetPosition) < Constants.MotorConstants.INTAKE_POS_TOLERANCE;
  }

  public double getTemperature() {
    return getMotor().getMotorTemperature();
  }

  // Hardware accessors
  public double getTargetPosition() {
    return targetPosition;
  }

  public void setMotorPositionToScoring() {
    getMotor().getEncoder().setPosition(Constants.MotorConstants.OUT_INTAKE_POS);
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
