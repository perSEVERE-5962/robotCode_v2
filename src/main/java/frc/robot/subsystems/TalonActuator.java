package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.atomic.AtomicReference;

public abstract class TalonActuator extends SubsystemBase implements Actuator {
  private TalonFX motor;
  private final VoltageOut m_voltageOut = new VoltageOut(0.0);
  private final PositionVoltage m_posVoltage = new PositionVoltage(0.0);
  private final VelocityVoltage m_velVoltage = new VelocityVoltage(0.0);

  private boolean isArm;
  private ExponentialProfile profile;
  private ArmFeedforward armFF;
  private ElevatorFeedforward elevatorFF;

  protected TalonActuator(
      int kID,
      double kP,
      double kI,
      double kD,
      double kMinOutput,
      double kMaxOutput,
      double kS,
      double kV,
      double kA,
      double kG,
      double kIz,
      double kUpperSoftLimit,
      double kLowerSoftLimit,
      double kGearRatio,
      int kStallLimit,
      boolean inverted,
      boolean coast,
      boolean useThroughBoreEncoder,
      boolean useSoftLimits,
      boolean isArm) {
    motor = new TalonFX(kID);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted =
        inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = coast ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = kStallLimit;

    // useThroughBoreEncoder ignored for now

    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;

    config.Feedback.SensorToMechanismRatio = kGearRatio;

    // iZone ignored because it doesn't seem to have it
    config.MotorOutput.PeakForwardDutyCycle = kMaxOutput;
    config.MotorOutput.PeakReverseDutyCycle = kMinOutput;
    config.Voltage.PeakForwardVoltage = kMaxOutput * 12.0;
    config.Voltage.PeakReverseVoltage = kMinOutput * 12.0;

    motor.setPosition(0);

    if (useSoftLimits) {
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kUpperSoftLimit;
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kLowerSoftLimit;
    }
    this.isArm = isArm;
    if (kA <= 0)
      kA = 0.00001; // 0 breaks the exponential profile because of internal division by zero
    profile =
        new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(12.0, kV, kA));
    if (isArm) {
      armFF = new ArmFeedforward(kS, kG, kV, kA);
    } else {
      elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
    }

    motor.getConfigurator().apply(config);
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public double getVelocity() {
    return motor.getVelocity().getValueAsDouble() * 60.0;
  }

  public void moveToPositionWithPID(double position) {
    motor.setControl(
        m_posVoltage
            .withPosition(position)
            .withFeedForward(isArm ? armFF.calculate(position, 0) : elevatorFF.calculate(0)));
  }

  public void moveToVelocityWithPID(double rpm) {
    motor.setControl(
        m_velVoltage
            .withVelocity(rpm / 60.0)
            .withFeedForward(
                isArm
                    ? armFF.calculate(Math.PI / 2, rpm / 60.0) // disable kG
                    : elevatorFF.calculate(rpm / 60.0))); // convert from RPM to RPS
  }

  public Command moveToGoalPosition(double goalPos) {
    AtomicReference<ExponentialProfile.State> currentState =
        new AtomicReference<>(new ExponentialProfile.State());
    final var goalState = new ExponentialProfile.State(goalPos, 0);
    return runEnd(
        () -> {
          var nextState = profile.calculate(0.02, currentState.get(), goalState);
          motor.setControl(
              m_posVoltage
                  .withPosition(nextState.position)
                  .withFeedForward(
                      isArm
                          ? armFF.calculateWithVelocities(
                              currentState.get().position,
                              currentState.get().velocity,
                              nextState.velocity)
                          : elevatorFF.calculateWithVelocities(
                              currentState.get().velocity, nextState.velocity)));
          currentState.set(nextState);
        },
        () -> {
          motor.set(0);
        });
  }

  public void move(double speed) {
    // Needed to do it like this because there is no voltage compensation
    motor.setControl(m_voltageOut.withOutput(speed * 12.0));
  }

  public TalonFX getMotor() {
    return motor;
  }

  public int getStickyFaultsRaw() {
    return motor.getFaultField().getValue();
  }

  public void updatePID(double kP, double kI, double kD, double kV) {
    Slot0Configs config = new Slot0Configs();
    motor.getConfigurator().refresh(config);
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    motor.getConfigurator().apply(config);
    if (isArm) {
      armFF.setKv(kV);
    } else {
      elevatorFF.setKv(kV);
    }
  }
}
