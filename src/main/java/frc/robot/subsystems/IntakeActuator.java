package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class IntakeActuator extends Actuator {
  private static IntakeActuator instance;
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private double targetPosition = 0;
  private static final double POSITION_TOLERANCE_ROTATIONS = 0.05;

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
        38.25f,
        11.5f,
        false,
        false,
        true);
        motor = getMotor();
    motorConfig = new SparkMaxConfig();

    motorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake).smartCurrentLimit(40);

    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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

  public double getAppliedOutput() {
    return getMotor().getAppliedOutput();
  }

  public double getOutputCurrent() {
    return getMotor().getOutputCurrent();
  }

  public static IntakeActuator getInstance() {
    if (instance == null) {
      instance = new IntakeActuator();
    }
    return instance;
  }
}
