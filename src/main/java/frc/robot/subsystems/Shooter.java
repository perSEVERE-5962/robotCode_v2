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

  // private SparkMax followerMotor;
  // private RelativeEncoder followerEncoder;
  // private SparkMaxConfig followerConfig;

  // private SparkMax followerMotor1;
  // private RelativeEncoder followerEncoder1;
  // private SparkMaxConfig followerConfig1;

  // private SparkMax followerMotor2;
  // private RelativeEncoder followerEncoder2;
  // private SparkMaxConfig followerConfig2;

  private double desiredRPM = 0;
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
        ShooterConstants.P,
        ShooterConstants.I,
        ShooterConstants.D,
        ShooterConstants.MinOutput,
        ShooterConstants.MaxOutput,
        ShooterConstants.FF,
        ShooterConstants.Iz,
        0,
        0,
        false,
        false,
        false);

    // and the constructor becomes:
    followers =
        new SparkMax[] {
          configureFollower(Constants.CANDeviceIDs.kShooterFollower, false), // same direction
          configureFollower(Constants.CANDeviceIDs.kShooterFollower1, true), // inverted
          configureFollower(Constants.CANDeviceIDs.kShooterFollower2, true), // inverted
        };
    motor = getMotor();

    motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    motorConfig.smartCurrentLimit(60);
    motorEncoder = motor.getEncoder();
    motorConfig.voltageCompensation(12.0);
    motorConfig.encoder.uvwMeasurementPeriod(8).uvwAverageDepth(2);
    // .quadratureMeasurementPeriod(8);
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    //     followers = new SparkMax[] {
    //     configureFollower(Constants.CANDeviceIDs.kShooterFollower, false),   // same direction
    //     configureFollower(CANDeviceIDs.kShooterFollower1, true),   // inverted
    //     configureFollower(CANDeviceIDs.kShooterFollower2, true),   // inverted
    // };

    // followerConfig = new SparkMaxConfig();
    // followerMotor = new SparkMax(Constants.CANDeviceIDs.kShooterFollower, MotorType.kBrushless);
    // followerConfig.follow(Constants.CANDeviceIDs.kShooterID, false);
    // followerConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    // followerConfig.smartCurrentLimit(60);
    // followerEncoder = followerMotor.getEncoder();
    // followerConfig.encoder
    //   .uvwMeasurementPeriod(8)
    //   .uvwAverageDepth(2);
    //   //.quadratureMeasurementPeriod(8);
    // followerMotor.configure(followerConfig, ResetMode.kNoResetSafeParameters,
    // PersistMode.kPersistParameters);

    // followerConfig1 = new SparkMaxConfig();
    // followerMotor1 = new SparkMax(Constants.CANDeviceIDs.kShooterFollower1,
    // MotorType.kBrushless);
    // followerConfig1.follow(Constants.CANDeviceIDs.kShooterID, true);
    // followerConfig1.idleMode(SparkBaseConfig.IdleMode.kCoast);
    // followerConfig1.smartCurrentLimit(60);
    // followerEncoder1 = followerMotor1.getEncoder();
    // followerConfig1.encoder
    //   .uvwMeasurementPeriod(8)
    //   .uvwAverageDepth(2);
    //   //.quadratureMeasurementPeriod(8);
    // followerMotor1.configure(followerConfig1, ResetMode.kNoResetSafeParameters,
    // PersistMode.kPersistParameters);

    // followerConfig2 = new SparkMaxConfig();
    // followerMotor2 = new SparkMax(Constants.CANDeviceIDs.kShooterFollower2,
    // MotorType.kBrushless);
    // followerConfig2.follow(Constants.CANDeviceIDs.kShooterID, true);
    // followerConfig2.idleMode(SparkBaseConfig.IdleMode.kCoast);
    // followerConfig2.smartCurrentLimit(60);
    // followerEncoder2 = followerMotor2.getEncoder();
    // followerConfig2.encoder
    //   .uvwMeasurementPeriod(8)
    //   .uvwAverageDepth(2);
    //   //.quadratureMeasurementPeriod(8);
    // followerMotor2.configure(followerConfig2, ResetMode.kNoResetSafeParameters,
    // PersistMode.kPersistParameters);

    limiter = new SlewRateLimiter(ShooterConstants.RPM_SLEW_RATE);
  }

  public double getVelocityRPM() {
    return motorEncoder.getVelocity() * ShooterConstants.VELOCITY_CONVERSION;
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
    // targetRPM = speed * 5700;
  }

  @Override
  public void moveToVelocityWithPID(double rpm) {
    this.desiredRPM = rpm;
    rpm = limiter.calculate(rpm);
    this.targetRPM = rpm;
    super.moveToVelocityWithPID(rpm);
  }

  private SparkMax configureFollower(int canId, boolean inverted) {
    SparkMax follower = new SparkMax(canId, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.follow(Constants.CANDeviceIDs.kShooterID, inverted);
    config.idleMode(SparkBaseConfig.IdleMode.kCoast);
    config.smartCurrentLimit(60);
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
