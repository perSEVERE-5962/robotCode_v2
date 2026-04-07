package frc.robot.subsystems;

import frc.robot.Constants;

public class IntakePivot extends TalonActuator {
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
        Constants.IntakePivotConstants.kG,
        Constants.IntakePivotConstants.kCosRatio,
        Constants.IntakePivotConstants.kIz,
        60f,
        11.5f,
        40,
        false,
        false,
        false,
        true,
        true);
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
    return getMotor().getDeviceTemp().getValueAsDouble();
  }

  // Hardware accessors
  public double getTargetPosition() {
    return targetPosition;
  }

  public void setMotorPositionToScoring() {
    getMotor().setPosition(Constants.MotorConstants.OUT_INTAKE_POS);
  }

  public double getAppliedOutput() {
    return getMotor().getDutyCycle().getValueAsDouble();
  }

  public double getOutputCurrent() {
    return getMotor().getStatorCurrent().getValueAsDouble();
  }

  public static IntakePivot getInstance() {
    if (instance == null) {
      instance = new IntakePivot();
    }
    return instance;
  }
}
