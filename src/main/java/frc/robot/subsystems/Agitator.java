package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.JamProtectionConstants;
import frc.robot.util.JamProtection;
import frc.robot.util.TunableNumber;

public class Agitator extends TalonActuator {
  private static Agitator instance;
  private static final TunableNumber kP =
      new TunableNumber("Agitator/kP", Constants.AgitatorConstants.kP);
  private static final TunableNumber kI =
      new TunableNumber("Agitator/kI", Constants.AgitatorConstants.kI);
  private static final TunableNumber kD =
      new TunableNumber("Agitator/kD", Constants.AgitatorConstants.kD);
  private static final TunableNumber kF =
      new TunableNumber("Agitator/FF", Constants.AgitatorConstants.kV);
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
    super(
        Constants.CANDeviceIDs.kAgitatorID,
        Constants.AgitatorConstants.kP,
        Constants.AgitatorConstants.kI,
        Constants.AgitatorConstants.kD,
        Constants.AgitatorConstants.kMinOutput,
        Constants.AgitatorConstants.kMaxOutput,
        Constants.AgitatorConstants.kS,
        Constants.AgitatorConstants.kV,
        0,
        1,
        Constants.AgitatorConstants.kIz,
        0,
        0,
        40,
        false,
        false,
        false,
        false,
        false);
  }

  public double getTemperature() {
    return getMotor().getDeviceTemp().getValueAsDouble();
  }

  public double getAppliedOutput() {
    return getMotor().getDutyCycle().getValueAsDouble();
  }

  public double getOutputCurrent() {
    return getMotor().getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    try {
      TunableNumber.ifChanged(
          () -> updatePID(kP.get(), kI.get(), kD.get(), kF.get()), kP, kI, kD, kF);
    } catch (Throwable t) {
      // CAN fault during PID update must not kill scheduler
    }

    // JamProtection detects and reports only. It never overrides the motor.
    // Telemetry reads the state; the driver decides what to do about it.
    try {
      jamProtection.update(getOutputCurrent(), getVelocity(), isRunning());
    } catch (Throwable t) {
      // CAN failure degrades jam detection, never kills drive control
    }
  }

  public boolean isRunning() {
    return Math.abs(getMotor().getDutyCycle().getValueAsDouble()) > 0.05;
  }

  public JamProtection getJamProtection() {
    return jamProtection;
  }

  public static double getTunableTargetRPM() {
    return targetSpeed.get();
  }

  public static double getJamCurrentThreshold() {
    return jamCurrentThreshold.get();
  }

  public static double getJamTimeThreshold() {
    return jamTimeThreshold.get();
  }

  // PID gain getters
  public static double getTunableKP() {
    return kP.get();
  }

  public static double getTunableKI() {
    return kI.get();
  }

  public static double getTunableKD() {
    return kD.get();
  }

  public static double getTunableFF() {
    return kF.get();
  }

  public static Agitator getInstance() {
    if (instance == null) {
      instance = new Agitator();
    }
    return instance;
  }
}
