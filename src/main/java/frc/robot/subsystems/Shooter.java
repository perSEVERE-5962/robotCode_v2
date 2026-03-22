package frc.robot.subsystems;

import com.revrobotics.PersistMode;
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
  private static final TunableNumber kP = new TunableNumber("Shooter/kP", ShooterConstants.kP);
  private static final TunableNumber kI = new TunableNumber("Shooter/kI", ShooterConstants.kI);
  private static final TunableNumber kD = new TunableNumber("Shooter/kD", ShooterConstants.kD);
  private static final TunableNumber kF = new TunableNumber("Shooter/FF", ShooterConstants.kV);

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
        Constants.ShooterConstants.kP,
        Constants.ShooterConstants.kI,
        Constants.ShooterConstants.kD,
        Constants.ShooterConstants.kMinOutput,
        Constants.ShooterConstants.kMaxOutput,
        Constants.ShooterConstants.kS,
        Constants.ShooterConstants.kV,
        0,
        0,
        1,
        Constants.ShooterConstants.kIz,
        0,
        0,
        60,
        false,
        true,
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

    limiter = new SlewRateLimiter(Constants.MotorConstants.SHOOTER_RPM_SLEW_RATE);
  }

  public boolean isAtSpeed() {
    if (desiredRPM == 0) return false;
    return Math.abs(desiredRPM - getVelocity()) < toleranceRPM.get();
  }

  @Override
  public void periodic() {
    TunableNumber.ifChanged(
        () -> updatePID(kP.get(), kI.get(), kD.get(), kF.get()), kP, kI, kD, kF);
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
