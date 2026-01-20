package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxAlternateEncoderSim;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends Actuator {
  private static Shooter instance;

  private SparkMax shooterMotor;

  private RelativeEncoder shooterMotorEncoder;
  private SparkMaxConfig motorConfig; 


  private Shooter() {
    super(Constants.CANDeviceIDs.kShooterID, 1.0, 0, 0, 0, 1.0, 0.0, 0, 0, 0, false, false, false);
    shooterMotor = new SparkMax(Constants.CANDeviceIDs.kShooterID, SparkLowLevel.MotorType.kBrushless);
    
    motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    motorConfig.smartCurrentLimit(40);
    motorConfig.encoder.velocityConversionFactor(1);
    shooterMotorEncoder = shooterMotor.getEncoder();
    
    shooterMotor.configure(motorConfig,ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }


  public double getMotorVelocity() {
    double velocity = shooterMotorEncoder.getVelocity();
    return velocity;
  }



  @Override
  public void periodic() {

  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }
}