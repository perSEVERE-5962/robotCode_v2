package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

// Profile code timing. Call reset() then record("name") between sections.
public class LoggedTracer {
  private LoggedTracer() {}

  private static double startTime = -1.0;

  public static void reset() {
    startTime = Timer.getFPGATimestamp();
  }

  // Logs time since last reset/record to LoggedTracer/{epochName}Ms
  public static void record(String epochName) {
    double now = Timer.getFPGATimestamp();
    if (startTime > 0) {
      double elapsedMs = (now - startTime) * 1000.0;
      Logger.recordOutput("LoggedTracer/" + epochName + "Ms", elapsedMs);
    }
    startTime = now;
  }
}
