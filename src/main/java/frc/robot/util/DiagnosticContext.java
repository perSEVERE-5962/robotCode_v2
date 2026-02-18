package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/** Crash-safe exception capture with stack traces and failure context. */
public final class DiagnosticContext {
  private static volatile String currentAction = "Idle";
  private static volatile String currentCommand = "None";
  private static volatile double lastExceptionTime = 0;
  private static volatile boolean exceptionDetected = false;

  private static final int MAX_STACK_FRAMES = 5;
  private static final String LOG_PREFIX = "Diagnostics/Exception/";

  private DiagnosticContext() {}

  /** Logs exception details. Called from uncaught exception handler. */
  public static void captureException(Thread thread, Throwable exception) {
    try {
      double timestamp = Timer.getFPGATimestamp();
      lastExceptionTime = timestamp;
      exceptionDetected = true;

      Logger.recordOutput(LOG_PREFIX + "Detected", true);
      Logger.recordOutput(LOG_PREFIX + "Timestamp", timestamp);
      Logger.recordOutput(LOG_PREFIX + "Class", safeGetClassName(exception));
      Logger.recordOutput(LOG_PREFIX + "Message", safeGetMessage(exception));
      Logger.recordOutput(LOG_PREFIX + "Thread", safeGetThreadName(thread));
      Logger.recordOutput(LOG_PREFIX + "StackTrace", formatStackTrace(exception));
      Logger.recordOutput(LOG_PREFIX + "Context", buildContext());
    } catch (Throwable t) {
    }
  }

  public static void setCurrentAction(String action) {
    try {
      currentAction = (action != null) ? action : "Unknown";
    } catch (Throwable t) {
    }
  }

  public static void setCurrentCommand(String command) {
    try {
      currentCommand = (command != null) ? command : "None";
    } catch (Throwable t) {
    }
  }

  public static synchronized String buildContext() {
    try {
      StringBuilder sb = new StringBuilder();
      sb.append(
          String.format(
              "Match: %s #%d @ %.1fs | ",
              safeGetMatchType(), safeGetMatchNumber(), safeGetMatchTime()));
      sb.append("Mode: ").append(safeGetMode()).append(" | ");
      sb.append("Action: ").append(currentAction).append(" | ");
      sb.append("Command: ").append(currentCommand).append(" | ");
      sb.append(
          String.format("Voltage: %.1fV | Loop: %.1fms", safeGetVoltage(), safeGetLoopTime()));
      return sb.toString();
    } catch (Throwable t) {
      return "Context unavailable";
    }
  }

  public static String formatStackTrace(Throwable exception) {
    try {
      if (exception == null) return "No exception";
      StackTraceElement[] stack = exception.getStackTrace();
      if (stack == null || stack.length == 0) return "No stack trace";

      StringBuilder sb = new StringBuilder();
      int framesToShow = Math.min(stack.length, MAX_STACK_FRAMES);

      for (int i = 0; i < framesToShow; i++) {
        if (i > 0) sb.append(" <- ");
        StackTraceElement frame = stack[i];
        sb.append(frame.getClassName())
            .append(".")
            .append(frame.getMethodName())
            .append(":")
            .append(frame.getLineNumber());
      }

      if (stack.length > MAX_STACK_FRAMES) {
        sb.append(" ... +").append(stack.length - MAX_STACK_FRAMES).append(" more");
      }
      return sb.toString();
    } catch (Throwable t) {
      return "Stack trace unavailable";
    }
  }

  public static void recordFailure(String failureType, String details) {
    try {
      double timestamp = Timer.getFPGATimestamp();
      Logger.recordOutput(
          "Diagnostics/FailureContext", failureType + ": " + details + " | " + buildContext());
      Logger.recordOutput("Diagnostics/FailureTimestamp", timestamp);
    } catch (Throwable t) {
    }
  }

  public static void clearException() {
    exceptionDetected = false;
  }

  public static boolean wasExceptionDetected() {
    return exceptionDetected;
  }

  public static double getLastExceptionTime() {
    return lastExceptionTime;
  }

  private static String safeGetClassName(Throwable t) {
    try {
      return (t != null) ? t.getClass().getName() : "Unknown";
    } catch (Throwable ex) {
      return "Unknown";
    }
  }

  private static String safeGetMessage(Throwable t) {
    try {
      String msg = (t != null) ? t.getMessage() : null;
      return (msg != null) ? msg : "No message";
    } catch (Throwable ex) {
      return "No message";
    }
  }

  private static String safeGetThreadName(Thread t) {
    try {
      return (t != null) ? t.getName() : "Unknown";
    } catch (Throwable ex) {
      return "Unknown";
    }
  }

  private static String safeGetMatchType() {
    try {
      return DriverStation.getMatchType().toString();
    } catch (Throwable t) {
      return "Unknown";
    }
  }

  private static int safeGetMatchNumber() {
    try {
      return DriverStation.getMatchNumber();
    } catch (Throwable t) {
      return 0;
    }
  }

  private static double safeGetMatchTime() {
    try {
      return DriverStation.getMatchTime();
    } catch (Throwable t) {
      return -1;
    }
  }

  private static String safeGetMode() {
    try {
      if (DriverStation.isDisabled()) return "Disabled";
      if (DriverStation.isAutonomous()) return "Auto";
      if (DriverStation.isTeleop()) return "Teleop";
      if (DriverStation.isTest()) return "Test";
      return "Unknown";
    } catch (Throwable t) {
      return "Unknown";
    }
  }

  private static double safeGetVoltage() {
    try {
      return RobotController.getBatteryVoltage();
    } catch (Throwable t) {
      return 0;
    }
  }

  private static double safeGetLoopTime() {
    try {
      return RobotController.getFPGATime() % 1000000 / 1000.0;
    } catch (Throwable t) {
      return 0;
    }
  }
}
