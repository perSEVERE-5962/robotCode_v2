// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.atomic.AtomicReference;

public abstract class FlexActuator extends SubsystemBase implements Actuator {
  private SparkFlex motor;
  private RelativeEncoder encoder;
  private SparkAbsoluteEncoder absoluteEncoder;
  private boolean useThroughBoreEncoder;
  private boolean isArm;

  private ExponentialProfile profile;
  private ElevatorFeedforward elevatorFF;
  private ArmFeedforward armFF;

  protected FlexActuator(
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

    motor = new SparkFlex(kID, SparkLowLevel.MotorType.kBrushless);
    SparkFlexConfig motorConfig = new SparkFlexConfig();

    motorConfig.inverted(inverted);
    motorConfig.idleMode(coast ? SparkBaseConfig.IdleMode.kCoast : SparkBaseConfig.IdleMode.kBrake);
    motorConfig.smartCurrentLimit(kStallLimit);
    motorConfig.voltageCompensation(12.0);
    motorConfig
        .encoder
        .quadratureAverageDepth(2)
        .quadratureMeasurementPeriod(10)
        .uvwMeasurementPeriod(8)
        .positionConversionFactor(kGearRatio)
        .velocityConversionFactor(kGearRatio);

    FeedbackSensor feedbackSensor = FeedbackSensor.kPrimaryEncoder;
    if (useThroughBoreEncoder) {
      feedbackSensor = FeedbackSensor.kAbsoluteEncoder;
    }
    motorConfig
        .closedLoop
        .feedbackSensor(feedbackSensor)
        .p(kP)
        .i(kI)
        .d(kD)
        .outputRange(kMinOutput, kMaxOutput)
        .iZone(kIz);

    if (useThroughBoreEncoder) {
      absoluteEncoder = motor.getAbsoluteEncoder();
    } else {
      encoder = motor.getEncoder();
      encoder.setPosition(0);
    }

    if (useSoftLimits) {

      SoftLimitConfig softLimitConfig = new SoftLimitConfig();
      softLimitConfig.forwardSoftLimitEnabled(true);
      softLimitConfig.forwardSoftLimit(kUpperSoftLimit);
      softLimitConfig.reverseSoftLimitEnabled(true);
      softLimitConfig.reverseSoftLimit(kLowerSoftLimit);

      motorConfig.apply(softLimitConfig);
    }
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.useThroughBoreEncoder = useThroughBoreEncoder;
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
  }

  public double getPosition() {
    if (useThroughBoreEncoder) {
      if (absoluteEncoder == null) {
        return 0;
      }
      return absoluteEncoder.getPosition();
    } else {
      if (encoder == null) {
        return 0;
      }
      return encoder.getPosition();
    }
  }

  public double getVelocity() {
    if (useThroughBoreEncoder) {
      if (absoluteEncoder == null) {
        return 0;
      }
      return absoluteEncoder.getVelocity();
    } else {
      if (encoder == null) {
        return 0;
      }
      return encoder.getVelocity();
    }
  }

  public void moveToPositionWithPID(double position) {
    motor
        .getClosedLoopController()
        .setSetpoint(
            position,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            isArm ? armFF.calculate(position, 0) : elevatorFF.calculate(0));
  }

  public void moveToVelocityWithPID(double rpm) {
    motor
        .getClosedLoopController()
        .setSetpoint(
            rpm,
            SparkBase.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            isArm ? armFF.calculate(Math.PI / 2, rpm) : elevatorFF.calculate(rpm)); // disable kG
  }

  public Command moveToGoalPosition(double goalPos) {
    AtomicReference<ExponentialProfile.State> currentState =
        new AtomicReference<>(new ExponentialProfile.State());
    final var goalState = new ExponentialProfile.State(goalPos, 0);
    return runEnd(
        () -> {
          var nextState = profile.calculate(0.02, currentState.get(), goalState);
          motor
              .getClosedLoopController()
              .setSetpoint(
                  nextState.position,
                  SparkBase.ControlType.kPosition,
                  ClosedLoopSlot.kSlot0,
                  isArm
                      ? armFF.calculateWithVelocities(
                          currentState.get().position,
                          currentState.get().velocity,
                          nextState.velocity)
                      : elevatorFF.calculateWithVelocities(
                          currentState.get().velocity, nextState.velocity));
          currentState.set(nextState);
        },
        () -> {
          motor.set(0);
        });
  }

  /* -1.0 <= speed <= 1.0 */
  public void move(double speed) {
    motor.set(speed);
  }

  public SparkFlex getMotor() {
    return motor;
  }

  /** Sticky faults as raw bits for diagnostics */
  public int getStickyFaultsRaw() {
    try {
      return (int) motor.getStickyFaults().rawBits;
    } catch (Throwable t) {
      return -1;
    }
  }

  /** Sticky warnings as raw bits. Paired with getStickyFaultsRaw for the fault decoder. */
  @Override
  public int getStickyWarningsRaw() {
    try {
      return (int) motor.getStickyWarnings().rawBits;
    } catch (Throwable t) {
      return -1;
    }
  }

  /** Hot-reload PID values. Creates new config, takes a few ms. */
  public void updatePID(double kP, double kI, double kD, double kV) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop.p(kP).i(kI).d(kD);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    if (isArm) {
      armFF.setKv(kV);
    } else {
      elevatorFF.setKv(kV);
    }
  }
}
