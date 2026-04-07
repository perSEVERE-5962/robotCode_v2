package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.util.TunableNumber;

public class IntakePivot extends TalonActuator {
  private static IntakePivot instance;

  private double targetPosition = 0;

  private static final TunableNumber kP =
      new TunableNumber("IntakePivot/kP", IntakePivotConstants.kP);
  private static final TunableNumber kI =
      new TunableNumber("IntakePivot/kI", IntakePivotConstants.kI);
  private static final TunableNumber kD =
      new TunableNumber("IntakePivot/kD", IntakePivotConstants.kD);
  private static final TunableNumber kF =
      new TunableNumber("IntakePivot/FF", IntakePivotConstants.kV);

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

  // @Override
  // public void moveToPositionWithPID(double position) {
  //   targetPosition = position;
  //   super.moveToPositionWithPID(position);
  // }

  public boolean isAtTarget() {
    return Math.abs(getPosition() - targetPosition) < Constants.MotorConstants.INTAKE_POS_TOLERANCE;
  }

  // public double getTemperature() {
  //   return getMotor().getMotorTemperature();
  // }

  // Hardware accessors
  public double getTargetPosition() {
    return targetPosition;
  }

  public void setMotorPositionToScoring() {
    getMotor().setPosition(Constants.MotorConstants.OUT_INTAKE_POS);
  }

  public void setStartingPose() {
    getMotor().setPosition(-0.4);
  }

  // public double getAppliedOutput() {
  //   return getMotor().getAppliedOutput();
  // }

  // public double getOutputCurrent() {
  //   return getMotor().getOutputCurrent();
  // }

  public double getTunableKP() {
    return kP.get();
  }

  public double getTunableKI() {
    return kI.get();
  }

  public double getTunableKD() {
    return kD.get();
  }

  public double getTunableFF() {
    return kF.get();
  }

  public void periodic() {
    try {
      TunableNumber.ifChanged(
          () -> updatePID(kP.get(), kI.get(), kD.get(), kF.get()), kP, kI, kD, kF);
    } catch (Throwable t) {
      // CAN fault during PID update must not kill scheduler
    }
  }

  public static IntakePivot getInstance() {
    if (instance == null) {
      instance = new IntakePivot();
    }
    return instance;
  }
}
