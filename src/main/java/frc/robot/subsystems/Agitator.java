package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.JamProtectionConstants;
import frc.robot.util.JamProtection;

public class Agitator extends Actuator {
  private SparkMax motor;
  private static Agitator instance;

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

  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  public double getOutputCurrent() {
    return motor.getOutputCurrent();
  }

  public double getVelocityRPM() {
    return motor.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    jamProtection.update(getOutputCurrent(), getVelocityRPM(), isRunning());
    double override = jamProtection.getMotorOverride();
    if (!Double.isNaN(override)) {
      motor.set(override);
    }
  }

  public boolean isRunning() {
    return Math.abs(motor.getAppliedOutput()) > 0.05;
  }

  public JamProtection getJamProtection() {
    return jamProtection;
  }

  public SparkMax getMotor() {
    return motor;
  }

  public int getStickyFaultsRaw() {
    try {
      return (int) motor.getStickyFaults().rawBits;
    } catch (Throwable t) {
      return -1;
    }
  }

  public static Agitator getInstance() {
    if (instance == null) {
      instance = new Agitator();
    }
    return instance;
  }
}
