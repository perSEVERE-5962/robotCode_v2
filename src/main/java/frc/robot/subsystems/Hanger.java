package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.HangerConstants;
import frc.robot.util.TunableNumber;

public class Hanger extends Actuator {
  private static Hanger instance;

  private double targetPosition = 0;

  // Tunable PID values
  private static final TunableNumber kP = new TunableNumber("Hanger/kP", HangerConstants.kP);
  private static final TunableNumber kD = new TunableNumber("Hanger/kD", HangerConstants.kD);

  private Hanger() {
    super(
        Constants.CANDeviceIDs.kHangerID,
        Constants.HangerConstants.kP,
        Constants.HangerConstants.kI,
        Constants.HangerConstants.kD,
        Constants.HangerConstants.kMinOutput,
        Constants.HangerConstants.kMaxOutput,
        Constants.HangerConstants.kS,
        Constants.HangerConstants.kV,
        Constants.HangerConstants.kG,
        0,
        1,
        Constants.HangerConstants.kIz,
        0,
        0,
        40,
        false,
        false,
        false,
        false,
        false);
  }

  @Override
  public void periodic() {
    TunableNumber.ifChanged(() -> updatePID(kP.get(), 0, kD.get(), 0), kP, kD);
  }

  @Override
  public void moveToPositionWithPID(double position) {
    this.targetPosition = position;
    super.moveToPositionWithPID(position);
  }

  public boolean isAtPosition() {
    return Math.abs(targetPosition - getPosition()) < Constants.MotorConstants.HANGER_POS_TOLERANCE;
  }

  // Hardware accessors
  public double getTargetPosition() {
    return targetPosition;
  }

  public double getAppliedOutput() {
    return getMotor().getAppliedOutput();
  }

  public double getOutputCurrent() {
    return getMotor().getOutputCurrent();
  }

  public double getTemperature() {
    return getMotor().getMotorTemperature();
  }

  public static Hanger getInstance() {
    if (instance == null) {
      instance = new Hanger();
    }
    return instance;
  }
}
