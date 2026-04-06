package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.util.TunableNumber;

public class IntakePivot extends TalonActuator {
  private static IntakePivot instance;
  private double targetPosition = 0;
  private static final double POSITION_TOLERANCE_ROTATIONS = 0.05;

  private static final TunableNumber kP =
      new TunableNumber("IntakePivot/kP", IntakePivotConstants.P);
  private static final TunableNumber kI =
      new TunableNumber("IntakePivot/kI", IntakePivotConstants.I);
  private static final TunableNumber kD =
      new TunableNumber("IntakePivot/kD", IntakePivotConstants.D);
  private static final TunableNumber kF =
      new TunableNumber("IntakePivot/FF", IntakePivotConstants.FF);

  private IntakePivot() {
    super(
        Constants.CANDeviceIDs.kIntakePivotID,
        Constants.IntakePivotConstants.kG,
        Constants.IntakePivotConstants.P,
        Constants.IntakePivotConstants.I,
        Constants.IntakePivotConstants.D,
        Constants.IntakePivotConstants.MinOutput,
        Constants.IntakePivotConstants.MaxOutput,
        Constants.IntakePivotConstants.FF,
        Constants.IntakePivotConstants.Iz,
        60f,
        11.5F,
        58.33,
        true,
        false,
        false);
    setStartingPose();
  }

  // @Override
  // public void moveToPositionWithPID(double position) {
  //   targetPosition = position;
  //   super.moveToPositionWithPID(position);
  // }

  public boolean isAtTarget() {
    return Math.abs(getPosition() - targetPosition) < POSITION_TOLERANCE_ROTATIONS;
  }

  // public double getTemperature() {
  //   return getMotor().getMotorTemperature();
  // }

  // Hardware accessors
  public double getTargetPosition() {
    return targetPosition;
  }

  public void setMotorPositionToScoring() {
    getMotor().setPosition(0.0);
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
