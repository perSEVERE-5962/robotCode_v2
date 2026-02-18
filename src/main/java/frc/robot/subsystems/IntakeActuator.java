package frc.robot.subsystems;

import frc.robot.Constants;

public class IntakeActuator extends Actuator {
  private static IntakeActuator instance;

  private IntakeActuator() {
    super(
        Constants.CANDeviceIDs.kIntakeActuatorID,
        Constants.IntakeConstants.P,
        Constants.IntakeConstants.I,
        Constants.IntakeConstants.D,
        Constants.IntakeConstants.MinOutput,
        Constants.IntakeConstants.MaxOutput,
        Constants.IntakeConstants.FF,
        Constants.IntakeConstants.Iz,
        0,
        0,
        false,
        false,
        false);
  }

  public static IntakeActuator getInstance() {
    if (instance == null) {
      instance = new IntakeActuator();
    }
    return instance;
  }
}
