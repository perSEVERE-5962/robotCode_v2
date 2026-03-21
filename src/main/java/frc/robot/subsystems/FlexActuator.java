// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlexActuator extends SubsystemBase {
  private SparkFlex motor;
  private RelativeEncoder encoder;
  private SparkAbsoluteEncoder absoluteEncoder;
  private boolean useThroughBoreEncoder;

  public FlexActuator(
      int kID,
      double kP,
      double kI,
      double kD,
      double kMinOutput,
      double kMaxOutput,
      double kF,
      double kIz,
      float kUpperSoftLimit,
      float kLowerSoftLimit,
      int kStallLimit,
      boolean inverted,
      boolean coast,
      boolean useThroughBoreEncoder,
      boolean useSoftLimits) {

    motor = new SparkFlex(kID, SparkLowLevel.MotorType.kBrushless);
    SparkFlexConfig motorConfig = new SparkFlexConfig();

    motorConfig.inverted(inverted);
    motorConfig.idleMode(coast ? SparkFlexConfig.IdleMode.kCoast : SparkFlexConfig.IdleMode.kBrake);
    motorConfig.smartCurrentLimit(kStallLimit);
    motorConfig.voltageCompensation(12.0);

    FeedbackSensor feedBackSensor = FeedbackSensor.kPrimaryEncoder;
    if (useThroughBoreEncoder) {
      feedBackSensor = FeedbackSensor.kAbsoluteEncoder;
    }
    motorConfig
        .closedLoop
        .feedbackSensor(feedBackSensor)
        .p(kP)
        .i(kI)
        .d(kD)
        .outputRange(kMinOutput, kMaxOutput)
        .iZone(kIz);
    motorConfig.closedLoop.feedForward.kV(12.0 * kF);
    motorConfig
        .encoder
        .quadratureAverageDepth(2)
        .quadratureMeasurementPeriod(10)
        .uvwMeasurementPeriod(8);
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

  /* -1 <= position <= 1 */
  public void moveToPositionWithPID(double position) {
    motor.getClosedLoopController().setSetpoint(position, SparkFlex.ControlType.kPosition);
  }

  public void moveToVelocityWithPID(double rpm) {
    motor.getClosedLoopController().setSetpoint(rpm, SparkFlex.ControlType.kVelocity);
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

  /** Hot-reload PID values. Creates new config, takes a few ms. */
  public void updatePID(double kP, double kI, double kD, double kF) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop.p(kP).i(kI).d(kD);
    config.closedLoop.feedForward.kV(12.0 * kF);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
