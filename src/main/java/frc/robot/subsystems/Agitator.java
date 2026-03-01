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
  private static final TunableNumber kP = new TunableNumber("Agitator/kP", AgitatorConstants.P);
  private static final TunableNumber kI = new TunableNumber("Agitator/kI", AgitatorConstants.I);
  private static final TunableNumber kD = new TunableNumber("Agitator/kD", AgitatorConstants.D);
  private static final TunableNumber kF = new TunableNumber("Agitator/FF", AgitatorConstants.FF);
private static final TunableNumber targetSpeed =
      new TunableNumber("Agitator/TargetSpeed", AgitatorConstants.TARGET_RPM);
  private static final TunableNumber jamCurrentThreshold =
      new TunableNumber("Agitator/JamAmps", AgitatorConstants.JAM_CURRENT_THRESHOLD_AMPS);
  private static final TunableNumber jamTimeThreshold =
      new TunableNumber("Agitator/JamSeconds", AgitatorConstants.JAM_TIME_THRESHOLD_SECONDS);
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
        Constants.AgitatorConstants.P,
        Constants.AgitatorConstants.I,
        Constants.AgitatorConstants.D,
        Constants.AgitatorConstants.MinOutput,
        Constants.AgitatorConstants.MaxOutput,
        Constants.AgitatorConstants.FF,
        Constants.AgitatorConstants.Iz,
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




  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  public double getOutputCurrent() {
    return motor.getOutputCurrent();
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
    // JamProtection detects and reports only. It never overrides the motor.
    // Telemetry reads the state; the driver decides what to do about it.
    try {
      jamProtection.update(getOutputCurrent(), getVelocityRPM(), isRunning());
    } catch (Throwable t) {
      // CAN failure degrades jam detection, never kills drive control
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
