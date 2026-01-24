package frc.robot.subsystems;

import frc.robot.Constants;

public class IntakeActuator extends Actuator {
  private static IntakeActuator instance;

  public IntakeActuator() {
    super(Constants.CANDeviceIDs.kIntakeActuatorID, 1.0, 0, 0, 0, 1.0, 0, 0, 0, 0, false, false, false);
  }

  public static IntakeActuator getInstance() {
    if (instance == null) {
      instance = new IntakeActuator();
    }
    return instance;
  }
}
