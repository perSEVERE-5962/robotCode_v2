package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.JamProtectionConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.util.JamProtection;
import frc.robot.util.TunableNumber;

public class Intake extends SubsystemBase {
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private static Intake instance;

  // Tunable operational value
  private static final TunableNumber intakeSpeed =
      new TunableNumber("Intake/Speed", MotorConstants.DESIRED_INTAKE_SPEED);

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

  private Intake() {
    motor = new SparkMax(Constants.CANDeviceIDs.kIntakeID, SparkLowLevel.MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    motorConfig.idleMode(SparkMaxConfig.IdleMode.kCoast).smartCurrentLimit(40);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void move(double speed) {
    motor.set(speed);
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

  public SparkMax getMotor() {
    return motor;
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

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }
}
