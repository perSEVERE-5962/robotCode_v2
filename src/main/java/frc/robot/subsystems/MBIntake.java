// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class MBIntake extends Actuator {
  private static MBIntake instance;
  private SparkMax followerMotor;
  private SparkMaxConfig followerConfig;

  private MBIntake() {
    super(
        IntakeConstants.kIntakeID,
        IntakeConstants.kP,
        IntakeConstants.kI,
        IntakeConstants.kD,
        IntakeConstants.kMinOutput,
        IntakeConstants.kMaxOutput,
        IntakeConstants.kFF,
        IntakeConstants.kIz,
        IntakeConstants.kUpperSoftLimit,
        IntakeConstants.kLowerSoftLimit,
        false,
        true,
        false);

    followerMotor = new SparkMax(IntakeConstants.kFollowerID, SparkLowLevel.MotorType.kBrushless);
    followerConfig = new SparkMaxConfig();
    followerConfig.follow(IntakeConstants.kIntakeID, true);
    followerConfig.inverted(true);
    followerConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    followerConfig.smartCurrentLimit(40);
    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // public void periodic() {
  // double theEncoder = instance.getPosition();
  // SmartDashboard.putNumber("Pivot", theEncoder);
  // }

  public static MBIntake getInstance() {
    if (instance == null) {
      instance = new MBIntake();
    }
    return instance;
  }
}