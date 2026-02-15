package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class EVMTest {

  @BeforeEach
  void setUp() {
    EventMarker.reset();
  }

  @Test
  void resetClearsState() {
    EventMarker.mark("TEST", "before");
    EventMarker.reset();

    assertEquals(0, EventMarker.getEventCount());
    assertEquals("", EventMarker.getLatestEvent());
    assertEquals("", EventMarker.getEventLog());
  }

  @Test
  void markUpdatesLatestAndLog() {
    EventMarker.mark("TEST", "one");

    assertEquals(1, EventMarker.getEventCount());
    assertEquals("one", EventMarker.getLatestEvent());

    String log = EventMarker.getEventLog();
    assertTrue(log.contains("TEST: one"));
    assertTrue(log.startsWith("["));
    assertTrue(log.contains("] "));
  }

  @Test
  void perCycleCapDropsEventsAndDoesNotUpdateLatest() {
    for (int i = 0; i < 50; i++) {
      EventMarker.mark("TEST", "e" + i);
    }

    assertEquals(50, EventMarker.getEventCount());
    String lastAccepted = EventMarker.getLatestEvent();

    EventMarker.mark("TEST", "dropped");

    assertEquals(50, EventMarker.getEventCount());
    assertEquals(lastAccepted, EventMarker.getLatestEvent());
  }

  @Test
  void flushAllowsNewCycleEvents() {
    for (int i = 0; i < 50; i++) {
      EventMarker.mark("TEST", "e" + i);
    }
    EventMarker.flushCycleEvents();
    EventMarker.mark("TEST", "after");

    assertEquals(51, EventMarker.getEventCount());
    assertEquals("after", EventMarker.getLatestEvent());
  }

  @Test
  void logIsBoundedToMaxEvents() {
    for (int i = 0; i < 120; i++) {
      EventMarker.mark("TEST", "event" + i);
      EventMarker.flushCycleEvents();
    }

    assertEquals(120, EventMarker.getEventCount());
    assertEquals(100, countLines(EventMarker.getEventLog()));
  }

  private static int countLines(String text) {
    if (text == null || text.isEmpty()) {
      return 0;
    }
    return text.split("\\R").length;
  }
}
