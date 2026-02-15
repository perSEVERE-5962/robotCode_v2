package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class EventMarkerTest {

  @BeforeEach
  void setUp() {
    HAL.initialize(500, 0);
    EventMarker.reset();
  }

  @Test
  void testInitialCountIsZero() {
    assertEquals(0, EventMarker.getEventCount());
  }

  @Test
  void testInitialLatestEventIsEmpty() {
    assertEquals("", EventMarker.getLatestEvent());
  }

  @Test
  void testInitialEventLogIsEmpty() {
    assertEquals("", EventMarker.getEventLog());
  }

  @Test
  void testMarkIncrementsCount() {
    EventMarker.mark("TEST", "one");
    assertEquals(1, EventMarker.getEventCount());
    EventMarker.mark("TEST", "two");
    assertEquals(2, EventMarker.getEventCount());
  }

  @Test
  void testMarkUpdatesLatestEvent() {
    EventMarker.mark("SHOT", "first");
    assertEquals("first", EventMarker.getLatestEvent());
    EventMarker.mark("INTAKE", "second");
    assertEquals("second", EventMarker.getLatestEvent());
  }

  @Test
  void testEventLogContainsFormattedEntry() {
    EventMarker.mark("SHOT", "Ball fired");
    String log = EventMarker.getEventLog();
    assertTrue(log.contains("SHOT: Ball fired"));
  }

  @Test
  void testEventLogContainsTimestamp() {
    EventMarker.mark("TEST", "timed");
    String log = EventMarker.getEventLog();
    assertTrue(log.matches(".*\\[\\d+\\.\\d+\\].*"), "Log entry should contain timestamp");
  }

  @Test
  void testShotFired() {
    EventMarker.shotFired();
    assertEquals("Ball fired", EventMarker.getLatestEvent());
    assertEquals(1, EventMarker.getEventCount());
  }

  @Test
  void testShotFiredWithNumber() {
    EventMarker.shotFired(5);
    assertEquals("Shot #5", EventMarker.getLatestEvent());
  }

  @Test
  void testBallIntaked() {
    EventMarker.ballIntaked();
    assertEquals("Ball acquired", EventMarker.getLatestEvent());
  }

  @Test
  void testBallEjected() {
    EventMarker.ballEjected();
    assertEquals("Ball ejected", EventMarker.getLatestEvent());
  }

  @Test
  void testAutoStart() {
    EventMarker.autoStart("ThreeShot");
    assertEquals("Started: ThreeShot", EventMarker.getLatestEvent());
  }

  @Test
  void testAutoWaypoint() {
    EventMarker.autoWaypoint(3);
    assertEquals("Waypoint 3", EventMarker.getLatestEvent());
  }

  @Test
  void testAutoComplete() {
    EventMarker.autoComplete();
    assertEquals("Completed", EventMarker.getLatestEvent());
  }

  @Test
  void testClimbStart() {
    EventMarker.climbStart();
    assertEquals("Climb initiated", EventMarker.getLatestEvent());
  }

  @Test
  void testClimbComplete() {
    EventMarker.climbComplete();
    assertEquals("Climb complete", EventMarker.getLatestEvent());
  }

  @Test
  void testVisionLock() {
    EventMarker.visionLock();
    assertEquals("Target locked", EventMarker.getLatestEvent());
  }

  @Test
  void testVisionLost() {
    EventMarker.visionLost();
    assertEquals("Target lost", EventMarker.getLatestEvent());
  }

  @Test
  void testModeChange() {
    EventMarker.modeChange("Autonomous");
    assertEquals("Autonomous", EventMarker.getLatestEvent());
  }

  @Test
  void testJamDetected() {
    EventMarker.jamDetected("Indexer");
    assertEquals("Indexer jam detected", EventMarker.getLatestEvent());
  }

  @Test
  void testJamCleared() {
    EventMarker.jamCleared("Indexer");
    assertEquals("Indexer jam cleared", EventMarker.getLatestEvent());
  }

  @Test
  void testEventLogBoundedToMaxEvents() {
    for (int i = 0; i < 120; i++) {
      EventMarker.mark("TEST", "event" + i);
      EventMarker.flushCycleEvents(); // flush each cycle to avoid per-cycle cap
    }
    // eventCount tracks total ever marked (unbounded counter)
    assertEquals(120, EventMarker.getEventCount());

    // But the log deque itself caps at 100
    String log = EventMarker.getEventLog();
    long lineCount = log.lines().count();
    assertTrue(lineCount <= 100, "Log deque should cap at 100 entries, got " + lineCount);
  }

  @Test
  void testPerCycleEventsAcceptedUpToLimit() {
    for (int i = 0; i < 50; i++) {
      EventMarker.mark("TEST", "cycle" + i);
    }
    assertEquals(50, EventMarker.getEventCount());
  }

  @Test
  void testPerCycleEventsDroppedPastLimit() {
    for (int i = 0; i < 55; i++) {
      EventMarker.mark("TEST", "cycle" + i);
    }
    // mark() returns early past 50 without incrementing count
    assertEquals(50, EventMarker.getEventCount());
  }

  @Test
  void testDroppedEventsDoNotUpdateLatest() {
    for (int i = 0; i < 50; i++) {
      EventMarker.mark("TEST", "accepted" + i);
    }
    String lastAccepted = EventMarker.getLatestEvent();

    // These should be dropped
    EventMarker.mark("TEST", "dropped51");
    EventMarker.mark("TEST", "dropped52");

    assertEquals(
        lastAccepted, EventMarker.getLatestEvent(), "Dropped events should not update latest");
  }

  @Test
  void testFlushCycleEventsDoesNotThrow() {
    assertDoesNotThrow(() -> EventMarker.flushCycleEvents());
  }

  @Test
  void testFlushEmptyCycleDoesNotThrow() {
    // No events marked, flush should be safe
    assertDoesNotThrow(() -> EventMarker.flushCycleEvents());
  }

  @Test
  void testFlushAllowsNewCycleEvents() {
    // Fill one cycle to capacity
    for (int i = 0; i < 50; i++) {
      EventMarker.mark("TEST", "fill" + i);
    }
    EventMarker.flushCycleEvents();

    // New cycle should accept events again
    EventMarker.mark("TEST", "afterFlush");
    assertEquals("afterFlush", EventMarker.getLatestEvent());
    assertEquals(51, EventMarker.getEventCount());
  }

  @Test
  void testResetClearsAll() {
    EventMarker.mark("TEST", "before reset");
    EventMarker.shotFired();

    EventMarker.reset();

    assertEquals(0, EventMarker.getEventCount());
    assertEquals("", EventMarker.getLatestEvent());
    assertEquals("", EventMarker.getEventLog());
  }

  @Test
  void testResetDoesNotThrow() {
    assertDoesNotThrow(() -> EventMarker.reset());
  }

  @Test
  void testResetAllowsNewEvents() {
    EventMarker.mark("TEST", "before");
    EventMarker.reset();
    EventMarker.mark("TEST", "after");

    assertEquals(1, EventMarker.getEventCount());
    assertEquals("after", EventMarker.getLatestEvent());
  }

  @Test
  void testCategoryConstantsExist() {
    assertNotNull(EventMarker.SHOT);
    assertNotNull(EventMarker.INTAKE);
    assertNotNull(EventMarker.AUTO);
    assertNotNull(EventMarker.CLIMB);
    assertNotNull(EventMarker.VISION);
    assertNotNull(EventMarker.ALERT);
    assertNotNull(EventMarker.MODE);
    assertNotNull(EventMarker.JAM);
  }
}
