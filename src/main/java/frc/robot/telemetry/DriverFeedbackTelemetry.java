package frc.robot.telemetry;

import frc.robot.util.DriverFeedback;

public class DriverFeedbackTelemetry implements SubsystemTelemetry {
  private DriverFeedback driverFeedback;

  private String activePattern = "none";
  private String priority = "NONE";
  private String description = "Idle";
  private double leftMotor = 0;
  private double rightMotor = 0;
  private double progressiveAimError = -1;
  private int patternCount = 0;

  public DriverFeedbackTelemetry() {
    this.driverFeedback = DriverFeedback.getInstance();
  }

  @Override
  public void update() {
    if (driverFeedback == null) {
      driverFeedback = DriverFeedback.getInstance();
    }
    if (driverFeedback == null) return;

    try {
      activePattern = driverFeedback.getActivePatternName();
      priority = driverFeedback.getActivePriority();
      description = driverFeedback.getPatternDescription();
      leftMotor = driverFeedback.getLeftMotor();
      rightMotor = driverFeedback.getRightMotor();
      progressiveAimError = driverFeedback.getProgressiveAimError();
      patternCount = driverFeedback.getPatternCount();
    } catch (Throwable t) {
      activePattern = "none";
      priority = "NONE";
      description = "Idle";
      leftMotor = 0;
      rightMotor = 0;
      progressiveAimError = -1;
      patternCount = 0;
    }
  }

  @Override
  public void log() {
    SafeLog.put("DriverFeedback/ActivePattern", activePattern);
    SafeLog.put("DriverFeedback/Priority", priority);
    SafeLog.put("DriverFeedback/Description", description);
    SafeLog.put("DriverFeedback/LeftMotor", leftMotor);
    SafeLog.put("DriverFeedback/RightMotor", rightMotor);
    SafeLog.put("DriverFeedback/ProgressiveAimError", progressiveAimError);
    SafeLog.put("DriverFeedback/PatternCount", patternCount);
  }

  @Override
  public String getName() {
    return "DriverFeedback";
  }
}
