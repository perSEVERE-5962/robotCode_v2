package frc.robot.subsystems;

import frc.robot.Constants;

public class Hanger extends Actuator {
  private static Hanger instance;

  private Hanger() {
    super(
        Constants.CANDeviceIDs.kHangerID,
        Constants.HangerConstants.P,
        Constants.HangerConstants.I,
        Constants.HangerConstants.D,
        Constants.HangerConstants.MinOutput,
        Constants.HangerConstants.MaxOutput,
        Constants.HangerConstants.FF,
        Constants.HangerConstants.Iz,
        0,
        0,
        false,
        false,
        false);
  }

  public static Hanger getInstance() {
    if (instance == null) {
      instance = new Hanger();
    }
    return instance;
  }
}
