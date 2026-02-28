package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.atomic.AtomicInteger;
import org.littletonrobotics.junction.Logger;

/** Lightweight event marker for AdvantageScope timeline. */
public class EventMarker {
  private static final int MAX_EVENTS = 100;
  private static final int MAX_CYCLE_EVENTS = 50; // Cap per-cycle events

  // Thread-safe collections
  private static final ConcurrentLinkedDeque<String> eventLog = new ConcurrentLinkedDeque<>();
  private static final AtomicInteger eventCount = new AtomicInteger(0);

  // Per-cycle event buffer (for batch logging)
  private static final List<String> currentCycleEvents = new ArrayList<>();
  private static int cycleEventCount = 0;
  private static int droppedEventsThisCycle = 0; // Track dropped events

  // Latest event info (volatile for thread safety)
  private static volatile String latestEvent = "";
  private static volatile String latestCategory = "";
  private static volatile double latestTimestamp = 0;

  // Event categories
  public static final String SHOT = "SHOT";
  public static final String INTAKE = "INTAKE";
  public static final String AUTO = "AUTO";
  public static final String CLIMB = "CLIMB";
  public static final String VISION = "VISION";
  public static final String ALERT = "ALERT";
  public static final String MODE = "MODE";
  public static final String JAM = "JAM";

  private EventMarker() {} // Static utility class

  /** Mark an event. */
  public static synchronized void mark(String category, String description) {
    try {
      // Check per-cycle cap first
      if (currentCycleEvents.size() >= MAX_CYCLE_EVENTS) {
        droppedEventsThisCycle++;
        return;
      }

      latestTimestamp = Timer.getFPGATimestamp();
      String event = String.format("[%.3f] %s: %s", latestTimestamp, category, description);

      latestEvent = description;
      latestCategory = category;
      eventLog.addLast(event);
      eventCount.incrementAndGet();

      // Keep log bounded
      while (eventLog.size() > MAX_EVENTS) {
        eventLog.pollFirst();
      }

      // Buffer for end-of-cycle batch logging
      currentCycleEvents.add(category + ": " + description);
      cycleEventCount++;
    } catch (Throwable t) {
    }
  }

  /**
   * Flush buffered events to log. Call once per cycle from TelemetryManager. All events from a
   * cycle get logged together as an array.
   */
  public static synchronized void flushCycleEvents() {
    try {
      if (droppedEventsThisCycle > 0) {
        currentCycleEvents.add(
            ALERT + ": Dropped " + droppedEventsThisCycle + " events (cap reached)");
      }

      if (!currentCycleEvents.isEmpty()) {
        Logger.recordOutput("Events/ThisCycle", currentCycleEvents.toArray(new String[0]));
        Logger.recordOutput("Events/ThisCycleCount", cycleEventCount);
      } else {
        Logger.recordOutput("Events/ThisCycle", new String[0]);
        Logger.recordOutput("Events/ThisCycleCount", 0);
      }
      Logger.recordOutput("Events/TotalCount", eventCount.get());
      Logger.recordOutput("Events/Latest", latestEvent);
      Logger.recordOutput("Events/LatestCategory", latestCategory);
      Logger.recordOutput("Events/DroppedThisCycle", droppedEventsThisCycle);
    } catch (Throwable t) {
    }

    currentCycleEvents.clear();
    cycleEventCount = 0;
    droppedEventsThisCycle = 0;
  }

  // ===== Shot events =====
  public static void shotFired() {
    mark(SHOT, "Ball fired");
  }

  public static void shotFired(int shotNumber) {
    mark(SHOT, "Shot #" + shotNumber);
  }

  // ===== Intake events =====
  public static void ballIntaked() {
    mark(INTAKE, "Ball acquired");
  }

  public static void ballEjected() {
    mark(INTAKE, "Ball ejected");
  }

  // ===== Auto events =====
  public static void autoStart(String autoName) {
    mark(AUTO, "Started: " + autoName);
  }

  public static void autoWaypoint(int num) {
    mark(AUTO, "Waypoint " + num);
  }

  public static void autoComplete() {
    mark(AUTO, "Completed");
  }

  // ===== Climb events =====
  public static void climbStart() {
    mark(CLIMB, "Climb initiated");
  }

  public static void climbComplete() {
    mark(CLIMB, "Climb complete");
  }

  // ===== Vision events =====
  public static void visionLock() {
    mark(VISION, "Target locked");
  }

  public static void visionLost() {
    mark(VISION, "Target lost");
  }

  // ===== Mode events =====
  public static void modeChange(String mode) {
    mark(MODE, mode);
  }

  // ===== Jam events =====
  public static void jamDetected(String subsystem) {
    mark(JAM, subsystem + " jam detected");
  }

  public static void jamCleared(String subsystem) {
    mark(JAM, subsystem + " jam cleared");
  }

  public static String getEventLog() {
    try {
      return String.join("\n", eventLog);
    } catch (Throwable t) {
      return "";
    }
  }

  public static String getLatestEvent() {
    return latestEvent;
  }

  public static int getEventCount() {
    return eventCount.get();
  }

  /** Reset for new match */
  public static synchronized void reset() {
    try {
      eventLog.clear();
      eventCount.set(0);
      currentCycleEvents.clear();
      cycleEventCount = 0;
      droppedEventsThisCycle = 0;
      latestEvent = "";
      latestCategory = "";
      latestTimestamp = 0;
    } catch (Throwable t) {
    }
  }
}
