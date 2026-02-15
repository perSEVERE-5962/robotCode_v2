package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class Shooter extends Actuator {
  private static Shooter instance;

  private SparkMax motor;
  private RelativeEncoder motorEncoder;
  private SparkMaxConfig motorConfig;

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
    motorConfig.encoder.velocityConversionFactor(1);
    motorEncoder = motor.getEncoder();

    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getMotorVelocity() {
    double velocity = motorEncoder.getVelocity();
    return velocity;
  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }
}
