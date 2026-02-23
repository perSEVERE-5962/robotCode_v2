package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

/**
 * Per-signal crash isolation for Logger.recordOutput() calls. One failure never affects other
 * signals or crashes the robot.
 */
public final class SafeLog {
  private static final AtomicInteger cycleFailures = new AtomicInteger(0);
  private static final AtomicReference<String> lastFailedKey = new AtomicReference<>("");

  private SafeLog() {}

  // =========================================================================
  // Primitive type logging
  // =========================================================================

  public static void put(String key, double value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
      recordFailure(key);
    }
  }

  public static void put(String key, boolean value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
      recordFailure(key);
    }
  }

  public static void put(String key, int value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
      recordFailure(key);
    }
  }

  public static void put(String key, long value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
      recordFailure(key);
    }
  }

  public static void put(String key, String value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
      recordFailure(key);
    }
  }

  // =========================================================================
  // Array type logging
  // =========================================================================

  public static void put(String key, double[] value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
      recordFailure(key);
    }
  }

  public static void put(String key, String[] value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
      recordFailure(key);
    }
  }

  public static void put(String key, int[] value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
      recordFailure(key);
    }
  }

  // =========================================================================
  // WPILib geometry types
  // =========================================================================

  public static void put(String key, Pose2d value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
      recordFailure(key);
    }
  }

  public static void put(String key, Pose3d value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
      recordFailure(key);
    }
  }

  public static void put(String key, Pose3d[] value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
      recordFailure(key);
    }
  }

  public static void put(String key, SwerveModuleState[] value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
      recordFailure(key);
    }
  }

  // =========================================================================
  // Action execution (for external calls like EventMarker, CycleTracker)
  // =========================================================================

  /** Run an action; swallow any exception. */
  public static void run(Runnable action) {
    try {
      action.run();
    } catch (Throwable t) {
      // swallow
    }
  }

  // =========================================================================
  // Failure tracking
  // =========================================================================

  private static void recordFailure(String key) {
    cycleFailures.incrementAndGet();
    lastFailedKey.set(key);
  }

  public static int getCycleFailures() {
    return cycleFailures.get();
  }

  public static String getLastFailedKey() {
    return lastFailedKey.get();
  }

  /** Log health metrics and reset. Call once per cycle after all telemetry. */
  public static void logAndReset() {
    try {
      int failures = cycleFailures.get();
      if (failures > 0) {
        Logger.recordOutput("Health/SafeLog/CycleFailures", failures);
        Logger.recordOutput("Health/SafeLog/LastFailedKey", lastFailedKey.get());
      }
    } catch (Throwable t) {
      // If even health logging fails, nothing we can do
    }
    cycleFailures.set(0);
    lastFailedKey.set("");
  }
}
