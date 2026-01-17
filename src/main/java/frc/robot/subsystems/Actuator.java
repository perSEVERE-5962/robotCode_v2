// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkBaseConfig;

public class Actuator extends SubsystemBase {
    private SparkMax armMotor;
    private SparkMaxConfig motorConfig;
    private RelativeEncoder armEncoder;
    private SparkAbsoluteEncoder absoluteEncoder;
    private boolean useThroughBoreEncoder = false;

    public Actuator(int ID, double P, double I, double D, double MinOutput, double MaxOutput, double FF, double Iz,
            float kUpperSoftLimit, float kLowerSoftLimit, boolean inverted, boolean useThroughBoreEncoder,
            boolean useSoftLimits) {

        armMotor = new SparkMax(ID, SparkLowLevel.MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();

        motorConfig.inverted(inverted);
        // Reach needs to be inverted
        // Wrist should not be inverted
        // Pivot should not be inverted
        motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        motorConfig.smartCurrentLimit(40);
        FeedbackSensor feedBackSensor = FeedbackSensor.kPrimaryEncoder;
        if (useThroughBoreEncoder == true) {
            feedBackSensor = FeedbackSensor.kAbsoluteEncoder;

        }
        motorConfig.closedLoop
                .feedbackSensor(feedBackSensor)
                .p(P)
                .i(I)
                .d(D)
                .outputRange(MinOutput, MaxOutput)
                .velocityFF(FF)
                .iZone(Iz);
        if (useThroughBoreEncoder == true) {
            absoluteEncoder = armMotor.getAbsoluteEncoder();
        } else {
            armEncoder = armMotor.getEncoder();
            armEncoder.setPosition(0);
        }

        if (useSoftLimits == true) {

            SoftLimitConfig softLimitConfig = new SoftLimitConfig();
            softLimitConfig.forwardSoftLimitEnabled(true);
            softLimitConfig.forwardSoftLimit(kUpperSoftLimit);
            softLimitConfig.reverseSoftLimitEnabled(true);
            softLimitConfig.reverseSoftLimit(kLowerSoftLimit);

            motorConfig.apply(softLimitConfig);
        }
        armMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.useThroughBoreEncoder = useThroughBoreEncoder;

    }

    public void periodic() {
        // nothing here
        motorConfig.closedLoop
                .p(getP())
                .i(getI())
                .d(getD());
    }

    public double getPosition() {
        if (useThroughBoreEncoder == true) {
            if (absoluteEncoder == null) {
                return 0;
            }
            return absoluteEncoder.getPosition();
        } else {
            if (armEncoder == null) {
                return 0;
            }
            return armEncoder.getPosition();
        }

    }

    public void moveToPositionWithPID(double position) {
        armMotor.getClosedLoopController().setReference(position, SparkMax.ControlType.kPosition);
    }

    public void move(double speed) {
        armMotor.set(speed);
    }

    public double getP() {
        return 0;
    }

    public double getI() {
        return 0;
    }

    public double getD() {
        return 0;
    }

    public SparkMax getArmMotor() {
        return armMotor;
    }
}
