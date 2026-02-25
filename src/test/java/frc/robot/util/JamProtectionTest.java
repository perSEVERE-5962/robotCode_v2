package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class JamProtectionTest {

  private JamProtection jp;
  private double fakeClock = 0;

  private static final double JAM_CURRENT = 25.0;
  private static final double JAM_VELOCITY = 100.0;
  private static final double STARTUP_IGNORE = 0.5;
  private static final double JAM_CONFIRM = 0.3;
  private static final double REVERSE_TIME = 0.4;
  private static final double COOLDOWN = 0.15;
  private static final double REVERSE_POWER = -0.3;
  private static final int MAX_ATTEMPTS = 3;

  @BeforeEach
  void setUp() {
    fakeClock = 0;
    jp =
        new JamProtection(
            "Test",
            JAM_CURRENT,
            JAM_VELOCITY,
            STARTUP_IGNORE,
            JAM_CONFIRM,
            REVERSE_TIME,
            COOLDOWN,
            REVERSE_POWER,
            MAX_ATTEMPTS,
            () -> fakeClock);
  }

  private void advanceTime(double seconds) {
    fakeClock += seconds;
  }

  @Test
  void testInitialStateIsMonitoring() {
    assertEquals(JamProtection.State.MONITORING, jp.getState());
    assertFalse(jp.isIntervening());
    assertFalse(jp.isDisabled());
    assertEquals(0, jp.getReverseAttempts());
  }

  @Test
  void testNoJamAtNormalOperation() {
    jp.update(10.0, 500.0, true);
    assertEquals(JamProtection.State.MONITORING, jp.getState());
    assertTrue(Double.isNaN(jp.getMotorOverride()), "No override during normal operation");
  }

  @Test
  void testNoJamWhenNotRunning() {
    jp.update(30.0, 0.0, false);
    assertEquals(JamProtection.State.MONITORING, jp.getState());
  }

  @Test
  void testStartupIgnoreSuppressesFalseJam() {
    // Motor starts running
    jp.update(30.0, 10.0, true);
    // Within startup ignore window, should stay MONITORING
    advanceTime(0.3); // still within 0.5s startup
    jp.update(30.0, 10.0, true);
    assertEquals(JamProtection.State.MONITORING, jp.getState());
  }

  @Test
  void testJamDetectedAfterStartupIgnore() {
    // Motor starts
    jp.update(5.0, 500.0, true);
    advanceTime(0.6); // past startup ignore

    // Jam condition
    jp.update(30.0, 10.0, true);
    assertEquals(JamProtection.State.JAM_CONFIRMING, jp.getState());
  }

  @Test
  void testJamClearsIfConditionClears() {
    // Past startup
    jp.update(5.0, 500.0, true);
    advanceTime(0.6);

    // Jam condition starts
    jp.update(30.0, 10.0, true);
    assertEquals(JamProtection.State.JAM_CONFIRMING, jp.getState());

    // Condition clears before confirmation
    advanceTime(0.1);
    jp.update(5.0, 500.0, true);
    assertEquals(JamProtection.State.MONITORING, jp.getState());
  }

  @Test
  void testJamConfirmationLeadsToReversing() {
    // Past startup
    jp.update(5.0, 500.0, true);
    advanceTime(0.6);

    // Jam detected
    jp.update(30.0, 10.0, true);
    assertEquals(JamProtection.State.JAM_CONFIRMING, jp.getState());

    // Sustain past confirmation time
    advanceTime(0.35);
    jp.update(30.0, 10.0, true);

    assertEquals(JamProtection.State.REVERSING, jp.getState());
    assertEquals(1, jp.getReverseAttempts());
    assertTrue(jp.isIntervening());
    assertEquals(REVERSE_POWER, jp.getMotorOverride(), 0.001);
  }

  @Test
  void testReversingTransitionsToCooldown() {
    // Get to REVERSING
    jp.update(5.0, 500.0, true);
    advanceTime(0.6);
    jp.update(30.0, 10.0, true);
    advanceTime(0.35);
    jp.update(30.0, 10.0, true);
    assertEquals(JamProtection.State.REVERSING, jp.getState());

    // Wait for reverse duration
    advanceTime(0.45);
    jp.update(5.0, 500.0, true);

    assertEquals(JamProtection.State.COOLDOWN, jp.getState());
    assertEquals(0, jp.getMotorOverride(), 0.001, "Motor should be stopped during cooldown");
    assertTrue(jp.isIntervening());
  }

  @Test
  void testCooldownReturnsToMonitoring() {
    // Get to COOLDOWN
    jp.update(5.0, 500.0, true);
    advanceTime(0.6);
    jp.update(30.0, 10.0, true);
    advanceTime(0.35);
    jp.update(30.0, 10.0, true);
    advanceTime(0.45);
    jp.update(5.0, 500.0, true);
    assertEquals(JamProtection.State.COOLDOWN, jp.getState());

    // Wait for cooldown
    advanceTime(0.2);
    jp.update(5.0, 500.0, true);

    assertEquals(JamProtection.State.MONITORING, jp.getState());
    assertFalse(jp.isIntervening());
  }

  @Test
  void testDisabledAfterMaxAttempts() {
    // MAX_ATTEMPTS reverses happen, then the next jam goes straight to DISABLED
    for (int attempt = 0; attempt <= MAX_ATTEMPTS; attempt++) {
      // Start motor / past startup ignore
      jp.update(5.0, 500.0, true);
      advanceTime(0.6);

      // Trigger jam confirmation
      jp.update(30.0, 10.0, true);
      advanceTime(0.35);
      jp.update(30.0, 10.0, true);

      if (attempt < MAX_ATTEMPTS) {
        assertEquals(
            JamProtection.State.REVERSING,
            jp.getState(),
            "Attempt " + attempt + " should reverse");
        // Let it reverse and cooldown
        advanceTime(0.45);
        jp.update(5.0, 500.0, true); // COOLDOWN
        advanceTime(0.2);
        jp.update(5.0, 500.0, true); // MONITORING
        assertEquals(JamProtection.State.MONITORING, jp.getState());
        advanceTime(0.6); // past new startup ignore
      }
    }

    assertEquals(JamProtection.State.DISABLED, jp.getState());
    assertTrue(jp.isDisabled());
    assertEquals(0, jp.getMotorOverride(), 0.001);
    assertEquals(MAX_ATTEMPTS, jp.getReverseAttempts());
  }

  @Test
  void testResetClearsDisabledState() {
    // Get to DISABLED (same pattern as testDisabledAfterMaxAttempts)
    for (int attempt = 0; attempt <= MAX_ATTEMPTS; attempt++) {
      jp.update(5.0, 500.0, true);
      advanceTime(0.6);
      jp.update(30.0, 10.0, true);
      advanceTime(0.35);
      jp.update(30.0, 10.0, true);
      if (attempt < MAX_ATTEMPTS) {
        advanceTime(0.45);
        jp.update(5.0, 500.0, true);
        advanceTime(0.2);
        jp.update(5.0, 500.0, true);
        advanceTime(0.6);
      }
    }
    assertTrue(jp.isDisabled());

    jp.reset();

    assertEquals(JamProtection.State.MONITORING, jp.getState());
    assertFalse(jp.isDisabled());
    assertEquals(0, jp.getReverseAttempts());
  }

  @Test
  void testMotorStopResetsToMonitoring() {
    // Get into jam confirmation
    jp.update(5.0, 500.0, true);
    advanceTime(0.6);
    jp.update(30.0, 10.0, true);
    assertEquals(JamProtection.State.JAM_CONFIRMING, jp.getState());

    // Motor stops (command ends)
    jp.update(0, 0, false);
    assertEquals(JamProtection.State.MONITORING, jp.getState());
  }

  @Test
  void testMotorStopDoesNotClearDisabled() {
    // Get to DISABLED
    for (int attempt = 0; attempt <= MAX_ATTEMPTS; attempt++) {
      jp.update(5.0, 500.0, true);
      advanceTime(0.6);
      jp.update(30.0, 10.0, true);
      advanceTime(0.35);
      jp.update(30.0, 10.0, true);
      if (attempt < MAX_ATTEMPTS) {
        advanceTime(0.45);
        jp.update(5.0, 500.0, true);
        advanceTime(0.2);
        jp.update(5.0, 500.0, true);
        advanceTime(0.6);
      }
    }
    assertTrue(jp.isDisabled());

    // Motor stop should NOT clear DISABLED
    jp.update(0, 0, false);
    assertTrue(jp.isDisabled(), "DISABLED should persist even when motor stops");
  }

  @Test
  void testHighCurrentAloneDoesNotTrigger() {
    // Past startup, high current but ALSO high velocity (not jammed, just working hard)
    jp.update(5.0, 500.0, true);
    advanceTime(0.6);
    jp.update(30.0, 500.0, true);
    assertEquals(JamProtection.State.MONITORING, jp.getState());
  }

  @Test
  void testLowVelocityAloneDoesNotTrigger() {
    // Past startup, low velocity but low current (motor coasting down)
    jp.update(5.0, 500.0, true);
    advanceTime(0.6);
    jp.update(5.0, 10.0, true);
    assertEquals(JamProtection.State.MONITORING, jp.getState());
  }

}
