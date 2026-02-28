package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import frc.robot.Constants.AgitatorConstants;
import frc.robot.util.TunableNumber;

public class Agitator extends Actuator {
  private SparkMax motor;
  private static Agitator instance;

  private static final TunableNumber kP = new TunableNumber("Agitator/kP", AgitatorConstants.P);
  private static final TunableNumber kI = new TunableNumber("Agitator/kI", AgitatorConstants.I);
  private static final TunableNumber kD = new TunableNumber("Agitator/kD", AgitatorConstants.D);
  private static final TunableNumber kF = new TunableNumber("Agitator/FF", AgitatorConstants.FF);

  private static final TunableNumber targetRPM =
      new TunableNumber("Agitator/TargetRPM", AgitatorConstants.TARGET_RPM);

  private Agitator() {
    // Actuator base class handles motor creation, PID, brake mode, and 40A current limit
    super(
        Constants.CANDeviceIDs.kAgitatorID,
        AgitatorConstants.P,
        AgitatorConstants.I,
        AgitatorConstants.D,
        AgitatorConstants.MinOutput,
        AgitatorConstants.MaxOutput,
        AgitatorConstants.FF,
        AgitatorConstants.Iz,
        0,
        0,
        false,
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

  public double getVelocityRPM() {
    return getMotorVelocity();
  }

  public boolean isRunning() {
    return Math.abs(motor.getAppliedOutput()) > 0.05;
  }

  public double getTunableTargetRPM() {
    return targetRPM.get();
  }

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

  public static Agitator getInstance() {
    if (instance == null) {
      instance = new Agitator();
    }
    return instance;
  }
}
