package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.TunableNumber;

public class Shooter extends Actuator {
  private static Shooter instance;
  private SparkMax motor;

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
        40,
        false,
        true,
        false,
        false);
    motor = getMotor();
  }

  public boolean isAtSpeed() {
    if (targetRPM == 0) return false;
    return Math.abs(targetRPM - getVelocity()) < toleranceRPM.get();
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
  public static double getTunableTargetRPM() {
    return targetRPMTunable.get();
  }

  public static double getToleranceRPM() {
    return toleranceRPM.get();
  }

  public static double getShotDropThreshold() {
    return shotDropRPM.get();
  }

  // PID gain getters
  public static double getTunableKP() {
    return kP.get();
  }

  public static double getTunableKI() {
    return kI.get();
  }

  public static double getTunableKD() {
    return kD.get();
  }

  public static double getTunableFF() {
    return kF.get();
  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }
}
