package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class CoprocessorHealthTest {

  private double mockTime = 0;
  private CoprocessorHealth health;

  @BeforeEach
  void setUp() {
    mockTime = 0;
    health = new CoprocessorHealth(() -> mockTime);
    health.register("back-left", "OrangePi1");
    health.register("back-right", "OrangePi1");
  }

  @Test
  void bothCamerasConnectedReadsAlive() {
    health.update(Map.of("back-left", true, "back-right", true));
    assertTrue(health.isGroupAlive("OrangePi1"));
    assertEquals(0, health.getRebootCount("OrangePi1"));
  }

  @Test
  void pairedDropAndRecoveryIncrementsRebootCount() {
    // Both alive at t=0
    mockTime = 0;
    health.update(Map.of("back-left", true, "back-right", true));

    // Both drop at t=10
    mockTime = 10;
    health.update(Map.of("back-left", false, "back-right", false));
    assertFalse(health.isGroupAlive("OrangePi1"));
    assertEquals(0, health.getRebootCount("OrangePi1"));

    // Both come back at t=30
    mockTime = 30;
    health.update(Map.of("back-left", true, "back-right", true));

    // Recovery dwell is 2s. Still pending at t=30.
    assertEquals(0, health.getRebootCount("OrangePi1"));

    // After 2 seconds of uninterrupted alive, the reboot is counted.
    mockTime = 32.5;
    health.update(Map.of("back-left", true, "back-right", true));
    assertEquals(1, health.getRebootCount("OrangePi1"));
    assertTrue(health.isGroupAlive("OrangePi1"));
  }

  @Test
  void singleCameraDropDoesNotIncrementRebootCount() {
    // Both alive at t=0
    mockTime = 0;
    health.update(Map.of("back-left", true, "back-right", true));

    // Left drops, right stays up (USB cable wiggle, not a Pi reboot)
    mockTime = 5;
    Map<String, Boolean> state = new HashMap<>();
    state.put("back-left", false);
    state.put("back-right", true);
    health.update(state);
    assertTrue(health.isGroupAlive("OrangePi1"));

    // Stays that way for a while
    mockTime = 20;
    health.update(state);
    assertTrue(health.isGroupAlive("OrangePi1"));
    assertEquals(0, health.getRebootCount("OrangePi1"));
  }

  @Test
  void sixRebootsInRowCountsCorrectly() {
    // Six WPI Day 1 style reboots
    for (int i = 0; i < 6; i++) {
      // Alive
      mockTime = i * 100.0;
      health.update(Map.of("back-left", true, "back-right", true));
      mockTime = i * 100.0 + 1;
      health.update(Map.of("back-left", true, "back-right", true));

      // Paired drop
      mockTime = i * 100.0 + 50;
      health.update(Map.of("back-left", false, "back-right", false));

      // Recovery
      mockTime = i * 100.0 + 70;
      health.update(Map.of("back-left", true, "back-right", true));
      // Wait through recovery dwell
      mockTime = i * 100.0 + 73;
      health.update(Map.of("back-left", true, "back-right", true));
    }
    assertEquals(6, health.getRebootCount("OrangePi1"));
    assertEquals(6, health.getTotalRebootCount());
  }

  @Test
  void anyRebootedRecentlyWindowCheck() {
    // Simulate one reboot at t=10-13
    mockTime = 0;
    health.update(Map.of("back-left", true, "back-right", true));
    mockTime = 10;
    health.update(Map.of("back-left", false, "back-right", false));
    mockTime = 11;
    health.update(Map.of("back-left", true, "back-right", true));
    mockTime = 13.5;
    health.update(Map.of("back-left", true, "back-right", true));
    assertEquals(1, health.getRebootCount("OrangePi1"));

    // Reboot timestamp was set around t=13.5
    mockTime = 20;
    assertTrue(health.anyRebootedRecently(10.0));

    mockTime = 100;
    assertFalse(health.anyRebootedRecently(10.0));
    assertTrue(health.anyRebootedRecently(200.0));
  }

  @Test
  void unknownGroupReturnsSafeDefaults() {
    assertFalse(health.isGroupAlive("NonExistentGroup"));
    assertEquals(0, health.getRebootCount("NonExistentGroup"));
    assertEquals(0.0, health.getLastRebootTimestamp("NonExistentGroup"), 0.0);
  }

  @Test
  void multipleGroupsAreIndependent() {
    health.register("front-left", "OrangePi2");
    health.register("front-right", "OrangePi2");

    Map<String, Boolean> state = new HashMap<>();
    state.put("back-left", true);
    state.put("back-right", true);
    state.put("front-left", false);
    state.put("front-right", false);

    mockTime = 0;
    health.update(state);
    assertTrue(health.isGroupAlive("OrangePi1"));
    assertFalse(health.isGroupAlive("OrangePi2"));
  }
}
