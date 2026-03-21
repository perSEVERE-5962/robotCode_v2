package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.JamProtectionConstants;
import frc.robot.util.JamProtection;
import frc.robot.util.TunableNumber;

public class Indexer extends FlexActuator {
  private SparkFlex motor;
  private static Indexer instance;

  // Tunable PID values
  private static final TunableNumber kP = new TunableNumber("Indexer/kP", IndexerConstants.P);
  private static final TunableNumber kI = new TunableNumber("Indexer/kI", IndexerConstants.I);
  private static final TunableNumber kD = new TunableNumber("Indexer/kD", IndexerConstants.D);
  private static final TunableNumber kF = new TunableNumber("Indexer/FF", IndexerConstants.FF);

  private final JamProtection jamProtection =
      new JamProtection(
          "Indexer",
          JamProtectionConstants.INDEXER_JAM_CURRENT_AMPS,
          JamProtectionConstants.INDEXER_JAM_VELOCITY_RPM,
          JamProtectionConstants.INDEXER_STARTUP_IGNORE_SEC,
          JamProtectionConstants.INDEXER_JAM_CONFIRM_SEC,
          JamProtectionConstants.INDEXER_REVERSE_SEC,
          JamProtectionConstants.INDEXER_COOLDOWN_SEC,
          JamProtectionConstants.INDEXER_REVERSE_POWER,
          JamProtectionConstants.INDEXER_MAX_ATTEMPTS);

  // Tunable operational values
  private static final TunableNumber targetSpeed =
      new TunableNumber("Indexer/TargetSpeed", IndexerConstants.TARGET_SPEED);
  private static final TunableNumber jamCurrentThreshold =
      new TunableNumber("Indexer/JamAmps", IndexerConstants.JAM_CURRENT_THRESHOLD_AMPS);
  private static final TunableNumber jamTimeThreshold =
      new TunableNumber("Indexer/JamSeconds", IndexerConstants.JAM_TIME_THRESHOLD_SECONDS);

  private Indexer() {
    super(
        Constants.CANDeviceIDs.kIndexerID,
        Constants.IndexerConstants.P,
        Constants.IndexerConstants.I,
        Constants.IndexerConstants.D,
        Constants.IndexerConstants.MinOutput,
        Constants.IndexerConstants.MaxOutput,
        Constants.IndexerConstants.FF,
        Constants.IndexerConstants.Iz,
        0,
        0,
        60,
        true,
        true,
        false,
        false);
    motor = getMotor();
  }

  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  @Override
  public void periodic() {
    TunableNumber.ifChanged(
        () -> updatePID(kP.get(), kI.get(), kD.get(), kF.get()), kP, kI, kD, kF);

    // JamProtection detects and reports only. It never overrides the motor.
    // Telemetry reads the state; the driver decides what to do about it.
    try {
      jamProtection.update(getOutputCurrent(), getVelocity(), isRunning());
    } catch (Throwable t) {
      // CAN failure degrades jam detection, never kills drive control
    }
  }

  public JamProtection getJamProtection() {
    return jamProtection;
  }

  // Hardware accessors
  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  public double getOutputCurrent() {
    return motor.getOutputCurrent();
  }

  public boolean isRunning() {
    return Math.abs(motor.getAppliedOutput()) > 0.05;
  }

  // Tunable accessors
  public static double getTunableTargetSpeed() {
    return targetSpeed.get();
  }

  public static double getJamCurrentThreshold() {
    return jamCurrentThreshold.get();
  }

  public static double getJamTimeThreshold() {
    return jamTimeThreshold.get();
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

  public static Indexer getInstance() {
    if (instance == null) {
      instance = new Indexer();
    }
    return instance;
  }
}
