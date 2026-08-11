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
  private static final TunableNumber kV =
      new TunableNumber("IntakePivot/kV", IntakePivotConstants.kV);
  private static final TunableNumber kG =
      new TunableNumber("IntakePivot/kG", IntakePivotConstants.kG);
  private static final TunableNumber kS =
      new TunableNumber("IntakePivot/kS", IntakePivotConstants.kS);

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
        Constants.IntakePivotConstants.kA,
        Constants.IntakePivotConstants.kG,
        Constants.IntakePivotConstants.kIz,
        Constants.MotorConstants.IN_INTAKE_POS,
        Constants.MotorConstants.OUT_INTAKE_POS,
        Constants.IntakePivotConstants.kGearRatio,
        58,
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

  /** Kraken device temperature in Celsius. */
  public double getTemperature() {
    return getMotor().getDeviceTemp().getValueAsDouble();
  }

  // Hardware accessors
  public double getTargetPosition() {
    return targetPosition;
  }

  public void setMotorPositionToScoring() {
    getMotor().setPosition(0.0);
  }

  /** Duty-cycle output, normalized -1 to 1, for parity with the Spark-based accessors. */
  public double getAppliedOutput() {
    return getMotor().getDutyCycle().getValueAsDouble();
  }

  /** Motor stator current in amps, read directly from the Kraken. */
  public double getOutputCurrent() {
    return getMotor().getStatorCurrent().getValueAsDouble();
  }

  /** Kraken supply voltage in volts. Used by telemetry to compute voltage drop. */
  public double getBusVoltage() {
    return getMotor().getSupplyVoltage().getValueAsDouble();
  }

  public double getTunableKP() {
    return kP.get();
  }

  public double getTunableKI() {
    return kI.get();
  }

  public double getTunableKD() {
    return kD.get();
  }

  public double getTunableKV() {
    return kV.get();
  }

  @Override
  public void periodic() {
    try {
      TunableNumber.ifChanged(
          () ->
              updatePID(
                  kP.get(),
                  kI.get(),
                  kD.get(),
                  kS.get(),
                  kV.get(),
                  Constants.IntakePivotConstants.kA,
                  kG.get()),
          kP,
          kI,
          kD,
          kS,
          kV,
          kG);
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
