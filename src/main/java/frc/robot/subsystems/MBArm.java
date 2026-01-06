// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class MBArm extends Actuator {
  private static MBArm instance;
  // private static AbsoluteEncoder absoluteEncoder;

  private MBArm() {
    super(
        ArmConstants.kArmID,
        ArmConstants.kP,
        ArmConstants.kI,
        ArmConstants.kD,
        ArmConstants.kMinOutput,
        ArmConstants.kMaxOutput,
        ArmConstants.kFF,
        ArmConstants.kIz,
        ArmConstants.kUpperSoftLimit,
        ArmConstants.kLowerSoftLimit,
        true,
        true,
        true);
  }

  public static MBArm getInstance() {
    if (instance == null) {
      instance = new MBArm();
    }
    return instance;
  }
}
