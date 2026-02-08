
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.util.TunableNumber;

public class Indexer extends Actuator {
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private static Indexer instance;

  // Tunable PID values
  private static final TunableNumber kP = new TunableNumber("Indexer/kP", IndexerConstants.P);
  private static final TunableNumber kI = new TunableNumber("Indexer/kI", IndexerConstants.I);
  private static final TunableNumber kD = new TunableNumber("Indexer/kD", IndexerConstants.D);
  private static final TunableNumber kF = new TunableNumber("Indexer/FF", IndexerConstants.FF);

  // Tunable operational values
  private static final TunableNumber targetSpeed =
          new TunableNumber("Indexer/TargetSpeed", IndexerConstants.TARGET_SPEED);
  private static final TunableNumber jamCurrentThreshold =
          new TunableNumber("Indexer/JamAmps", IndexerConstants.JAM_CURRENT_THRESHOLD_AMPS);
  private static final TunableNumber jamTimeThreshold =
          new TunableNumber("Indexer/JamSeconds", IndexerConstants.JAM_TIME_THRESHOLD_SECONDS);

  private Indexer() {
    super(Constants.CANDeviceIDs.kIndexerID, Constants.IndexerConstants.P, Constants.IndexerConstants.I,
            Constants.IndexerConstants.D, Constants.IndexerConstants.MinOutput, Constants.IndexerConstants.MaxOutput,
            Constants.IndexerConstants.FF, Constants.IndexerConstants.Iz, 0, 0, true, false, false);
    motor = getMotor();
    motorConfig = new SparkMaxConfig();

    motorConfig
            .idleMode(SparkMaxConfig.IdleMode.kCoast)
            .smartCurrentLimit(20);

    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  @Override
  public void periodic() {
    TunableNumber.ifChanged(
        () -> updatePID(kP.get(), kI.get(), kD.get(), kF.get()),
        kP, kI, kD, kF);
  }

  // Hardware accessors
  public double getAppliedOutput() { return motor.getAppliedOutput(); }
  public double getOutputCurrent() { return motor.getOutputCurrent(); }
  public double getVelocityRPM() { return getMotorVelocity(); }
  public boolean isRunning() { return Math.abs(motor.getAppliedOutput()) > 0.05; }

  // Tunable accessors
  public double getTunableTargetSpeed() { return targetSpeed.get(); }
  public double getJamCurrentThreshold() { return jamCurrentThreshold.get(); }
  public double getJamTimeThreshold() { return jamTimeThreshold.get(); }

  // PID gain getters
  public double getTunableKP() { return kP.get(); }
  public double getTunableKI() { return kI.get(); }
  public double getTunableKD() { return kD.get(); }
  public double getTunableFF() { return kF.get(); }

  public static Indexer getInstance() {
    if (instance == null) {
      instance = new Indexer();
    }
    return instance;
  }
}
