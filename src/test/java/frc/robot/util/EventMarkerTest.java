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
  void testShotFired() {
    EventMarker.shotFired();
    assertEquals("Ball fired", EventMarker.getLatestEvent());
    assertEquals(1, EventMarker.getEventCount());
  }

  @Test
  void testEventLogBoundedToMaxEvents() {
    for (int i = 0; i < 120; i++) {
      EventMarker.mark("TEST", "event" + i);
      EventMarker.flushCycleEvents();
    }
    assertEquals(120, EventMarker.getEventCount());

    String log = EventMarker.getEventLog();
    long lineCount = log.lines().count();
    assertTrue(lineCount <= 100, "Log deque should cap at 100 entries, got " + lineCount);
  }

  @Test
  void testPerCycleEventsDroppedPastLimit() {
    for (int i = 0; i < 55; i++) {
      EventMarker.mark("TEST", "cycle" + i);
    }
    assertEquals(50, EventMarker.getEventCount());
  }

  @Test
  void testFlushAllowsNewCycleEvents() {
    for (int i = 0; i < 50; i++) {
      EventMarker.mark("TEST", "fill" + i);
    }
    EventMarker.flushCycleEvents();

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
}
