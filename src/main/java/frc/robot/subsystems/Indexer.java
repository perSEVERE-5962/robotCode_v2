package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private static Indexer instance;

  public Indexer() {
    motor = new SparkMax(Constants.CANDeviceIDs.kIndexerID, SparkLowLevel.MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    motorConfig
      //.inverted(true)
      .idleMode(SparkMaxConfig.IdleMode.kCoast)
      .smartCurrentLimit(40);
    
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void move(double speed) {
    motor.set(speed);
  }

  public static Indexer getInstance() {
    if (instance == null) {
      instance = new Indexer();
    }
    return instance;
  }
}
