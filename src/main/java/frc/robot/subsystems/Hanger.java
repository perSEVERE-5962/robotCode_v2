package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.HangerConstants;
import frc.robot.util.TunableNumber;

public class Hanger extends Actuator {
  private static Hanger instance;

  private double targetPosition = 0;

  // Tunable PID values
  private static final TunableNumber kP = new TunableNumber("Hanger/kP", HangerConstants.P);
  private static final TunableNumber kD = new TunableNumber("Hanger/kD", HangerConstants.D);

  private Hanger() {
    super(
        Constants.CANDeviceIDs.kHangerID,
        Constants.HangerConstants.P,
        Constants.HangerConstants.I,
        Constants.HangerConstants.D,
        Constants.HangerConstants.MinOutput,
        Constants.HangerConstants.MaxOutput,
        Constants.HangerConstants.FF,
        Constants.HangerConstants.Iz,
        0,
        0,
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
    return Math.abs(targetPosition - getPosition()) < HangerConstants.POSITION_TOLERANCE;
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
