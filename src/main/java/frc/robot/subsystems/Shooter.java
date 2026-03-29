package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.TunableNumber;

public class Shooter extends Actuator {
  private static Shooter instance;
  private SlewRateLimiter limiter;
  private SparkMax[] followers;

  private SparkMax motor;
  private RelativeEncoder motorEncoder;
  private SparkMaxConfig motorConfig;

  private double desiredRPM = 0;

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
        ShooterConstants.P,
        ShooterConstants.I,
        ShooterConstants.D,
        ShooterConstants.MinOutput,
        ShooterConstants.MaxOutput,
        ShooterConstants.FF,
        ShooterConstants.Iz,
        0,
        0,
        true,
        false,
        false);

    motor = getMotor();
    motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    motorConfig.smartCurrentLimit(30);
    motorEncoder = motor.getEncoder();
    motorConfig.voltageCompensation(12.0);
    motorConfig.encoder.uvwMeasurementPeriod(8).uvwAverageDepth(2);
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    followers =
        new SparkMax[] {
          configureFollower(Constants.CANDeviceIDs.kShooterFollower, false),
          configureFollower(Constants.CANDeviceIDs.kShooterFollower1, true),
          configureFollower(Constants.CANDeviceIDs.kShooterFollower2, true),
        };

    // limiter = new SlewRateLimiter(ShooterConstants.RPM_SLEW_RATE);
  }

  public double getVelocityRPM() {
    return motorEncoder.getVelocity() * ShooterConstants.VELOCITY_CONVERSION;
  }

  public boolean isAtSpeed(double wantedRpm) {
    if (wantedRpm == 0) return false;
    return Math.abs(wantedRpm - getVelocityRPM()) < toleranceRPM.get();
  }

  public boolean isAtSpeed() {
    if (desiredRPM == 0) return false;
    return Math.abs(desiredRPM - getVelocityRPM()) < toleranceRPM.get();
  }

  @Override
  public void periodic() {
    try {
      TunableNumber.ifChanged(
          () -> updatePID(kP.get(), kI.get(), kD.get(), kF.get()), kP, kI, kD, kF);
    } catch (Throwable t) {
      // CAN fault during PID update must not kill scheduler
    }
  }

  public void move(double speed) {
    motor.set(speed);
  }

  // @Override
  // public void moveToVelocityWithPID(double rpm) {
  //   this.desiredRPM = rpm;
  //   motor.getClosedLoopController().setSetpoint(rpm, SparkMax.ControlType.kVelocity);
  // }

  private SparkMax configureFollower(int canId, boolean inverted) {
    SparkMax follower = new SparkMax(canId, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.follow(Constants.CANDeviceIDs.kShooterID, inverted);
    config.idleMode(SparkBaseConfig.IdleMode.kCoast);
    config.smartCurrentLimit(30);
    config.voltageCompensation(12.0);
    config.encoder.uvwMeasurementPeriod(8).uvwAverageDepth(2);
    follower.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    return follower;
  }

  public double getMotorVelocity() {
    return getVelocityRPM();
  }

  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  // Hardware accessors
  public double getTargetRPM() {
    return desiredRPM;
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
