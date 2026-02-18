package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class Indexer extends Actuator {
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private static Indexer instance;

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
        true,
        false,
        false);
    motor = /*new SparkMax(Constants.CANDeviceIDs.kIndexerID, SparkLowLevel.MotorType.kBrushless);*/
        getMotor();
    motorConfig = new SparkMaxConfig();

    motorConfig
        /*.inverted(true)*/
        .idleMode(SparkMaxConfig.IdleMode.kCoast)
        .smartCurrentLimit(20);

    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  /*public void move(double speed) {
    motor.set(speed);
  }*/

  public static Indexer getInstance() {
    if (instance == null) {
      instance = new Indexer();
    }
    return instance;
  }
}
