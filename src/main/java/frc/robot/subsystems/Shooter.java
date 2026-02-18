package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.TunableNumber;

public class Shooter extends Actuator {
  private static Shooter instance;

  private SparkMax motor;
  private RelativeEncoder motorEncoder;
  private SparkMaxConfig motorConfig;

  private double targetRPM = 0;

  // Tunable PID values
  private static final TunableNumber kP = new TunableNumber("Shooter/kP", ShooterConstants.P);
  private static final TunableNumber kI = new TunableNumber("Shooter/kI", ShooterConstants.I);
  private static final TunableNumber kD = new TunableNumber("Shooter/kD", ShooterConstants.D);
  private static final TunableNumber kF = new TunableNumber("Shooter/FF", ShooterConstants.FF);

  // Tunable setpoints and thresholds
  private static final TunableNumber targetRPMTunable =
      new TunableNumber("Shooter/TargetRPM", ShooterConstants.TARGET_RPM);
  private static final TunableNumber toleranceRPM =
      new TunableNumber("Shooter/ToleranceRPM", ShooterConstants.SPEED_TOLERANCE_RPM);
  private static final TunableNumber shotDropRPM =
      new TunableNumber("Shooter/ShotDropRPM", ShooterConstants.SHOT_DETECTION_DROP_RPM);

  private Shooter() {
    super(
        Constants.CANDeviceIDs.kShooterID,
        Constants.ShooterConstants.P,
        Constants.ShooterConstants.I,
        Constants.ShooterConstants.D,
        Constants.ShooterConstants.MinOutput,
        Constants.ShooterConstants.MaxOutput,
        Constants.ShooterConstants.FF,
        Constants.ShooterConstants.Iz,
        0,
        0,
        false,
        false,
        false);
    motor = getMotor();

    motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    motorConfig.smartCurrentLimit(40);
    motorEncoder = motor.getEncoder();

    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getVelocityRPM() {
    return motorEncoder.getVelocity() * ShooterConstants.VELOCITY_CONVERSION;
  }

  public boolean isAtSpeed() {
    if (targetRPM == 0) return false;
    return Math.abs(targetRPM - getVelocityRPM()) < toleranceRPM.get();
  }

  @Override
  public void periodic() {
    TunableNumber.ifChanged(
        () -> updatePID(kP.get(), kI.get(), kD.get(), kF.get()), kP, kI, kD, kF);
  }

  @Override
  public void moveToVelocityWithPID(double rpm) {
    this.targetRPM = rpm;
    super.moveToVelocityWithPID(rpm);
  }

  public double getMotorVelocity() {
    return getVelocityRPM();
  }

  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  // Hardware accessors
  public double getTargetRPM() {
    return targetRPM;
  }

  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  public double getOutputCurrent() {
    return motor.getOutputCurrent();
  }

  public double getBusVoltage() {
    return motor.getBusVoltage();
  }

  // Tunable accessors
  public double getTunableTargetRPM() {
    return targetRPMTunable.get();
  }

  public double getToleranceRPM() {
    return toleranceRPM.get();
  }

  public double getShotDropThreshold() {
    return shotDropRPM.get();
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

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }
}
