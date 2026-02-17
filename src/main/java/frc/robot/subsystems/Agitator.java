package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class Agitator extends SubsystemBase {
  private SparkFlex motor;
  private SparkFlexConfig motorConfig;
  private static Agitator instance;

  private Agitator() {
    motor = new SparkFlex(Constants.CANDeviceIDs.kAgitatorID, SparkLowLevel.MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();

    motorConfig
      //.inverted(true)
      .idleMode(SparkFlexConfig.IdleMode.kBrake)
      .smartCurrentLimit(40);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void move(double speed) {
    motor.set(speed);
  }

  public static Agitator getInstance() {
    if (instance == null) {
      instance = new Agitator();
    }
    return instance;
  }

}
