package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import frc.robot.Constants;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.Constants.JamProtectionConstants;
import frc.robot.util.JamProtection;
import frc.robot.util.TunableNumber;

public class IntakeRoller extends FlexActuator {
  private SparkFlex motor;
  private static IntakeRoller instance;

  // Tunable operational value
  private static final TunableNumber intakeRollerSpeed =
      new TunableNumber("IntakeRoller/Speed", IntakeRollerConstants.TARGET_SPEED);

  private final JamProtection jamProtection =
      new JamProtection(
          "IntakeRoller",
          JamProtectionConstants.INTAKE_JAM_CURRENT_AMPS,
          JamProtectionConstants.INTAKE_JAM_VELOCITY_RPM,
          JamProtectionConstants.INTAKE_STARTUP_IGNORE_SEC,
          JamProtectionConstants.INTAKE_JAM_CONFIRM_SEC,
          JamProtectionConstants.INTAKE_REVERSE_SEC,
          JamProtectionConstants.INTAKE_COOLDOWN_SEC,
          JamProtectionConstants.INTAKE_REVERSE_POWER,
          JamProtectionConstants.INTAKE_MAX_ATTEMPTS);

  private IntakeRoller() {
    super(
        Constants.CANDeviceIDs.kIntakeRollerID,
        Constants.IntakeRollerConstants.kP,
        Constants.IntakeRollerConstants.kI,
        Constants.IntakeRollerConstants.kD,
        Constants.IntakeRollerConstants.kMinOutput,
        Constants.IntakeRollerConstants.kMaxOutput,
        Constants.IntakeRollerConstants.kS,
        Constants.IntakeRollerConstants.kV,
        0,
        1,
        Constants.IntakeRollerConstants.kIz,
        0,
        0,
        50,
        true,
        true,
        false,
        false,
        false);
    motor = getMotor();
  }

  @Override
  public void periodic() {
    // JamProtection detects and reports only. It never overrides the motor.
    // Telemetry reads the state; the driver decides what to do about it.
    try {
      jamProtection.update(getOutputCurrent(), getVelocity(), isRunning());
    } catch (Throwable t) {
      // CAN failure degrades jam detection, never kills drive control
    }
  }

  public JamProtection getJamProtection() {
    return jamProtection;
  }

  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  // Hardware accessors
  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  public double getOutputCurrent() {
    return motor.getOutputCurrent();
  }

  public double getBusVoltage() {
    return motor.getBusVoltage();
  }

  public boolean isRunning() {
    return Math.abs(motor.getAppliedOutput()) > 0.05;
  }

  // Tunable value accessor
  public static double getTunableSpeed() {
    return intakeRollerSpeed.get();
  }

  /** Sticky faults as raw bits for diagnostics */
  public int getStickyFaultsRaw() {
    try {
      return (int) motor.getStickyFaults().rawBits;
    } catch (Throwable t) {
      return -1;
    }
  }

  public static IntakeRoller getInstance() {
    if (instance == null) {
      instance = new IntakeRoller();
    }
    return instance;
  }
}
