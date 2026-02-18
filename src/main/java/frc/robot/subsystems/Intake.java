package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.util.TunableNumber;

public class Intake extends SubsystemBase {
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private static Intake instance;

  // Tunable operational value
  private static final TunableNumber intakeSpeed =
      new TunableNumber("Intake/Speed", MotorConstants.DESIRED_INTAKE_SPEED);

  private Intake() {
    motor = new SparkMax(Constants.CANDeviceIDs.kIntakeID, SparkLowLevel.MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    motorConfig
        // .inverted(true)
        .idleMode(SparkMaxConfig.IdleMode.kCoast)
        .smartCurrentLimit(40);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void move(double speed) {
    motor.set(speed);
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
