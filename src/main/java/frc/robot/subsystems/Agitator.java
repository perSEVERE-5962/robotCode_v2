package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.JamProtectionConstants;
import frc.robot.util.JamProtection;
import frc.robot.util.TunableNumber;

public class Agitator extends Actuator {
  private static Agitator instance;
  private static final TunableNumber kP = new TunableNumber("Agitator/kP", Constants.AgitatorConstants.P);
  private static final TunableNumber kI = new TunableNumber("Agitator/kI", Constants.AgitatorConstants.I);
  private static final TunableNumber kD = new TunableNumber("Agitator/kD", Constants.AgitatorConstants.D);
  private static final TunableNumber kF = new TunableNumber("Agitator/FF", Constants.AgitatorConstants.FF);
private static final TunableNumber targetSpeed =
      new TunableNumber("Agitator/TargetSpeed", Constants.AgitatorConstants.TARGET_RPM);
  private static final TunableNumber jamCurrentThreshold =
      new TunableNumber("Agitator/JamAmps", Constants.AgitatorConstants.JAM_CURRENT_THRESHOLD_AMPS);
  private static final TunableNumber jamTimeThreshold =
      new TunableNumber("Agitator/JamSeconds", Constants.AgitatorConstants.JAM_TIME_THRESHOLD_SECONDS);
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
  }

  public double getTemperature() {
    return getMotor().getMotorTemperature();
  }




  public double getAppliedOutput() {
    return getMotor().getAppliedOutput();
  }

  public double getOutputCurrent() {
    return getMotor().getOutputCurrent();
  }

  public double getVelocityRPM() {
    return getMotor().getEncoder().getVelocity();
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
    return Math.abs(getMotor().getAppliedOutput()) > 0.05;
  }

  public JamProtection getJamProtection() {
    return jamProtection;
  }

public double getTunableTargetRPM() {
    return targetSpeed.get();
  }

  public double getJamCurrentThreshold() {
    return jamCurrentThreshold.get();
  }

  public double getJamTimeThreshold() {
    return jamTimeThreshold.get();
  }

  // PID gain getters
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
