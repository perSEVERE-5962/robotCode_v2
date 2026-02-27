package frc.robot.subsystems;

import frc.robot.Constants;

public class Shooter extends Actuator {
  private static Shooter instance;

  private Shooter() {
    super(Constants.CANDeviceIDs.kShooterID, Constants.ShooterConstants.P, Constants.ShooterConstants.I, Constants.ShooterConstants.D, Constants.ShooterConstants.MinOutput, Constants.ShooterConstants.MaxOutput, Constants.ShooterConstants.FF, Constants.ShooterConstants.Iz, 0, 0, 40, false, true, false, false);
  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }
}