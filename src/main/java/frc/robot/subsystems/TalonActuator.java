package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TalonActuator extends SubsystemBase implements Actuator {
  private TalonFX motor;
  private final VoltageOut m_voltageOut = new VoltageOut(0.0);
  private final PositionVoltage m_posVoltage = new PositionVoltage(0.0);
  private final VelocityVoltage m_velVoltage = new VelocityVoltage(0.0);

  protected TalonActuator(
      int kID,
      double kP,
      double kI,
      double kD,
      double kMinOutput,
      double kMaxOutput,
      double kF,
      double kIz,
      double kUpperSoftLimit,
      double kLowerSoftLimit,
      boolean inverted,
      boolean useThroughBoreEncoder,
      boolean useSoftLimits) {
    motor = new TalonFX(kID);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted =
        inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40; // bad default for krakens; 120 is the default

    // useThroughBoreEncoder ignored for now
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kV = 12.0 * kF;
    // iZone ignored because it doesn't seem to have it
    config.MotorOutput.PeakForwardDutyCycle = kMaxOutput;
    config.MotorOutput.PeakReverseDutyCycle = kMinOutput;
    config.Voltage.PeakForwardVoltage = kMaxOutput / 12.0;
    config.Voltage.PeakReverseVoltage = kMinOutput / 12.0;

    motor.setPosition(0);

    if (useSoftLimits) {
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kUpperSoftLimit;
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kLowerSoftLimit;
    }
    motor.getConfigurator().apply(config);
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public double getMotorVelocity() {
    return motor.getVelocity().getValueAsDouble() * 60.0;
  }

  public void moveToPositionWithPID(double position) {
    motor.setControl(m_posVoltage.withPosition(position));
  }

  public void moveToVelocityWithPID(double rpm) {
    motor.setControl(m_velVoltage.withVelocity(rpm / 60.0)); // convert from RPM to RPS
  }

  public void move(double speed) {
    // Needed to do it like this because there is no voltage compensation
    motor.setControl(m_voltageOut.withOutput(speed * 12.0)); // TODO: test negative speeds
  }

  public TalonFX getMotor() {
    return motor;
  }

  public int getStickyFaultsRaw() {
    return motor.getFaultField().getValue();
  }

  public void updatePID(double kP, double kI, double kD, double kF) {
    Slot0Configs config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    config.kV = 12.0 * kF;
    motor.getConfigurator().apply(config); // Note: Overrides all Slot0 config values already there
  }
}
