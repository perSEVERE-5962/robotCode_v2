package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.util.TunableNumber;

public class IntakeActuator extends Actuator {
  private static IntakeActuator instance;

  private double targetPosition = 0;

  // Tunable PID values
  private static final TunableNumber kP =
      new TunableNumber("IntakeActuator/kP", IntakeConstants.P);
  private static final TunableNumber kI =
      new TunableNumber("IntakeActuator/kI", IntakeConstants.I);
  private static final TunableNumber kD =
      new TunableNumber("IntakeActuator/kD", IntakeConstants.D);
  private static final TunableNumber kF =
      new TunableNumber("IntakeActuator/FF", IntakeConstants.FF);

  // Tunable position targets
  private static final TunableNumber deployPosition =
      new TunableNumber("IntakeActuator/DeployPos", MotorConstants.OUT_INTAKE_POS);
  private static final TunableNumber retractPosition =
      new TunableNumber("IntakeActuator/RetractPos", MotorConstants.IN_INTAKE_POS);
  private static final TunableNumber positionTolerance =
      new TunableNumber("IntakeActuator/Tolerance", 0.05);

  private IntakeActuator() {
    super(
        Constants.CANDeviceIDs.kIntakeActuatorID,
        Constants.IntakeConstants.P,
        Constants.IntakeConstants.I,
        Constants.IntakeConstants.D,
        Constants.IntakeConstants.MinOutput,
        Constants.IntakeConstants.MaxOutput,
        Constants.IntakeConstants.FF,
        Constants.IntakeConstants.Iz,
        0,
        0,
        false,
        false,
        false);
  }

  @Override
  public void periodic() {
    TunableNumber.ifChanged(
        () -> updatePID(kP.get(), kI.get(), kD.get(), kF.get()), kP, kI, kD, kF);
  }

  @Override
  public void moveToPositionWithPID(double position) {
    targetPosition = position;
    super.moveToPositionWithPID(position);
  }

  public boolean isAtTarget() {
    return Math.abs(getPosition() - targetPosition) < positionTolerance.get();
  }

  public double getTemperature() {
    return getMotor().getMotorTemperature();
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

  // Tunable position accessors
  public double getTunableDeployPosition() {
    return deployPosition.get();
  }

  public double getTunableRetractPosition() {
    return retractPosition.get();
  }

  // PID gain getters
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

  public static IntakeActuator getInstance() {
    if (instance == null) {
      instance = new IntakeActuator();
    }
    return instance;
  }
}
