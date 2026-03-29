package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Constants.JamProtectionConstants;
import frc.robot.util.JamProtection;
import frc.robot.util.TunableNumber;

public class Agitator extends SubsystemBase{
  private TalonFX motor ;
  private TalonFXConfiguration motorConfig;
  private static Agitator instance;
  private VelocityVoltage rVelocityVoltageuest;
  private static final TunableNumber kP =
      new TunableNumber("Agitator/kP", Constants.AgitatorConstants.P);
  private static final TunableNumber kI =
      new TunableNumber("Agitator/kI", Constants.AgitatorConstants.I);
  private static final TunableNumber kD =
      new TunableNumber("Agitator/kD", Constants.AgitatorConstants.D);
  private static final TunableNumber kF =
      new TunableNumber("Agitator/FF", Constants.AgitatorConstants.FF);
  private static final TunableNumber targetSpeed =
      new TunableNumber("Agitator/TargetSpeed", Constants.AgitatorConstants.TARGET_RPM);
  private static final TunableNumber jamCurrentThreshold =
      new TunableNumber("Agitator/JamAmps", Constants.AgitatorConstants.JAM_CURRENT_THRESHOLD_AMPS);
  private static final TunableNumber jamTimeThreshold =
      new TunableNumber(
          "Agitator/JamSeconds", Constants.AgitatorConstants.JAM_TIME_THRESHOLD_SECONDS);
  private final JamProtection jamProtection =
      new JamProtection(
          "Agitator",
          JamProtectionConstants.AGITATOR_JAM_CURRENT_AMPS,
          JamProtectionConstants.AGITATOR_JAM_VELOCITY_RPM,
          JamProtectionConstants.AGITATOR_STARTUP_IGNORE_SEC,
          JamProtectionConstants.AGITATOR_JAM_CONFIRM_SEC,
          JamProtectionConstants.AGITATOR_REVERSE_SEC,
          JamProtectionConstants.AGITATOR_COOLDOWN_SEC,
          JamProtectionConstants.AGITATOR_REVERSE_POWER,
          JamProtectionConstants.AGITATOR_MAX_ATTEMPTS);

  private Agitator() {
    // Actuator base class handles motor creation, PID, brake mode, and 40A current limit
    // super(
    //     Constants.CANDeviceIDs.kAgitatorID,
    //     Constants.AgitatorConstants.P,
    //     Constants.AgitatorConstants.I,
    //     Constants.AgitatorConstants.D,
    //     Constants.AgitatorConstants.MinOutput,
    //     Constants.AgitatorConstants.MaxOutput,
    //     Constants.AgitatorConstants.FF,
    //     Constants.AgitatorConstants.Iz,
    //     0,
    //     0,
    //     true,
    //     false,
    //     false);

    motor = new TalonFX(51);
    motorConfig = new TalonFXConfiguration();
    rVelocityVoltageuest = new VelocityVoltage(0);

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    motorConfig.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    motorConfig.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    motorConfig.Slot0.kP = 0.15; // An error of 1 rotation per second results in 0.11 V output
    motorConfig.Slot0.kI = 0; // No output for integrated error
    motorConfig.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    motor.getConfigurator().apply(motorConfig);

    
   // motorConfig.voltageCompensation(12.0);
    
  }

  public double getTemperature() {
     return 0.0;
  }

  public double getAppliedOutput() {
    return 0.0;
  }

  public double getOutputCurrent() {
    return 0.0;
  }

  public double getVelocityRPM() {
    return 0.0;
  }


  public void runVelocity(){
    motor.setControl(rVelocityVoltageuest.withVelocity((-1*(6000.0/60))));
  }

  public void runVelocityReverse(){
    motor.setControl(rVelocityVoltageuest.withVelocity(((6000.0/60))));
  }

  public void stopVelocity(){
    motor.setControl(rVelocityVoltageuest.withVelocity(0));
  }

  //@Override
  // public void periodic() {
  //   // JamProtection detects and reports only. It never overrides the motor.
  //   // Telemetry reads the state; the driver decides what to do about it.
  //   try {
  //     jamProtection.update(getOutputCurrent(), getVelocityRPM(), isRunning());
  //   } catch (Throwable t) {
  //     // CAN failure degrades jam detection, never kills drive control
  //   }
  // }

  public boolean isRunning() {
    return true;
  }

  public JamProtection getJamProtection() {
    return jamProtection;
  }

  public double getTunableTargetRPM() {
    return 0.0;
  }

  public double getJamCurrentThreshold() {
    return 0.0;
  }

  public double getJamTimeThreshold() {
    return 0.0;
  }

  // PID gain getters
  public double getTunableKP() {
    return 0.0;
  }

  public double getTunableKI() {
   return 0.0;
  }

  public double getTunableKD() {
    return 0.0;
  }

  public double getTunableFF() {
    return kF.get();
  }

  public static Agitator getInstance() {
    if (instance == null) {
      instance = new Agitator();
    }
    return instance;
  }
}
