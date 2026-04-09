package frc.robot.telemetry;

import frc.robot.Constants;
import frc.robot.util.LEDStatusDisplay;

public class LEDTelemetry implements SubsystemTelemetry {
  private LEDStatusDisplay ledDisplay;

  private String state = "DISABLED";
  private String description = "Normal";
  private boolean hardwareAvailable = false;
  private double currentBrightness = 0;
  private int stateChangeCount = 0;

  public LEDTelemetry() {
    this.ledDisplay = LEDStatusDisplay.getInstance();
  }

  @Override
  public void update() {
    if (ledDisplay == null) {
      ledDisplay = LEDStatusDisplay.getInstance();
    }
    if (ledDisplay == null) return;

    try {
      state = ledDisplay.getCurrentStateName();
      description = ledDisplay.getStateDescription();
      hardwareAvailable = ledDisplay.isHardwareAvailable();
      currentBrightness = ledDisplay.getCurrentBrightness();
      stateChangeCount = ledDisplay.getStateChangeCount();
    } catch (Throwable t) {
      state = "DISABLED";
      description = "Normal";
      hardwareAvailable = false;
      currentBrightness = 0;
      stateChangeCount = 0;
    }
  }

  @Override
  public void log() {
    SafeLog.put("LED/State", state);
    SafeLog.put("LED/Device/Connected", hardwareAvailable);
    SafeLog.put("LED/StateChangeCount", stateChangeCount);

    if (Constants.TUNING_MODE) {
      SafeLog.put("LED/Brightness", currentBrightness);
      SafeLog.put("LED/Description", description);
    }
  }

  @Override
  public String getName() {
    return "LED";
  }
}
