package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;
import frc.robot.Constants.JamProtectionConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.util.JamProtection;
import frc.robot.util.TunableNumber;

public class IntakeRoller extends FlexActuator {
  private SparkFlex motor;
  private SparkFlexConfig motorConfig;
  private static IntakeRoller instance;

  // Tunable operational value
  private static final TunableNumber intakeSpeed =
      new TunableNumber("Intake/Speed", MotorConstants.DESIRED_INTAKE_RPM);

  private final JamProtection jamProtection =
      new JamProtection(
          "Intake",
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
        Constants.IntakeRollerConstants.P,
        Constants.IntakeRollerConstants.I,
        Constants.IntakeRollerConstants.D,
        Constants.IntakeRollerConstants.MinOutput,
        Constants.IntakeRollerConstants.MaxOutput,
        Constants.IntakeRollerConstants.FF,
        Constants.IntakeRollerConstants.Iz,
        0,
        0,
        true,
        false,
        false);
    motor = getMotor();

    motorConfig = new SparkFlexConfig();

    motorConfig.idleMode(SparkFlexConfig.IdleMode.kCoast).smartCurrentLimit(50);

    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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

  public double getVelocityRPM() {
    return motor.getEncoder().getVelocity();
  }

  public boolean isRunning() {
    return Math.abs(motor.getAppliedOutput()) > 0.05;
  }

  // Tunable value accessor
  public double getTunableSpeed() {
    return intakeSpeed.get();
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
