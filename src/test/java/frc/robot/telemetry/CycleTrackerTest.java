package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class CycleTrackerTest extends TelemetryTestBase {

  private CycleTracker tracker;

  @BeforeEach
  void setUp() {
    tracker = CycleTracker.getInstance();
    tracker.reset();
  }

  @Test
  void testInitialState() {
    assertEquals(CycleTracker.CyclePhase.IDLE, tracker.getCurrentPhase());
    assertEquals(0, tracker.getCompletedCycles());
    assertEquals(0, tracker.getFailedCycles());
    assertEquals(0, tracker.getLastTotalCycleTimeMs(), 0.001);
  }

  @Test
  void testFullCycleHappyPath() throws Exception {
    tracker.intakeStarted();
    assertEquals(CycleTracker.CyclePhase.INTAKING, tracker.getCurrentPhase());
    Thread.sleep(10);

    tracker.intakeComplete();
    assertEquals(CycleTracker.CyclePhase.INDEXING, tracker.getCurrentPhase());
    Thread.sleep(10);

    tracker.indexComplete();
    assertEquals(CycleTracker.CyclePhase.AIMING, tracker.getCurrentPhase());
    Thread.sleep(10);

    tracker.aimComplete();
    assertEquals(CycleTracker.CyclePhase.SHOOTING, tracker.getCurrentPhase());
    Thread.sleep(10);

    tracker.shotFired();
    assertEquals(CycleTracker.CyclePhase.RECOVERING, tracker.getCurrentPhase());
    Thread.sleep(10);

    tracker.recoveryComplete();
    assertEquals(CycleTracker.CyclePhase.IDLE, tracker.getCurrentPhase());

    assertEquals(1, tracker.getCompletedCycles());
    assertEquals(0, tracker.getFailedCycles());
    assertTrue(tracker.getLastTotalCycleTimeMs() > 0, "Total cycle time should be positive");
  }

  @Test
  void testAbortFromIntaking() {
    tracker.intakeStarted();
    assertEquals(CycleTracker.CyclePhase.INTAKING, tracker.getCurrentPhase());

    tracker.cycleAborted("ball lost");
    assertEquals(CycleTracker.CyclePhase.IDLE, tracker.getCurrentPhase());
    assertEquals(1, tracker.getFailedCycles());
    assertEquals(0, tracker.getCompletedCycles());
  }

  @Test
  void testAbortFromAiming() throws Exception {
    tracker.intakeStarted();
    Thread.sleep(5);
    tracker.intakeComplete();
    Thread.sleep(5);
    tracker.indexComplete();
    assertEquals(CycleTracker.CyclePhase.AIMING, tracker.getCurrentPhase());

    tracker.cycleAborted("vision lost");
    assertEquals(CycleTracker.CyclePhase.IDLE, tracker.getCurrentPhase());
    assertEquals(1, tracker.getFailedCycles());
    assertEquals(0, tracker.getCompletedCycles());
  }

  @Test
  void testAbortFromIdle() {
    assertEquals(CycleTracker.CyclePhase.IDLE, tracker.getCurrentPhase());
    tracker.cycleAborted("spurious abort");

    // Should do nothing when already IDLE
    assertEquals(0, tracker.getFailedCycles());
    assertEquals(CycleTracker.CyclePhase.IDLE, tracker.getCurrentPhase());
  }

  @Test
  void testMultipleCycles() throws Exception {
    for (int i = 0; i < 3; i++) {
      runCompleteCycle();
    }

    assertEquals(3, tracker.getCompletedCycles());
    assertEquals(0, tracker.getFailedCycles());

    double bestMs = getField(tracker, "bestCycleTimeMs");
    double worstMs = getField(tracker, "worstCycleTimeMs");
    assertTrue(bestMs > 0, "Best cycle time should be tracked");
    assertTrue(worstMs >= bestMs, "Worst should be >= best");
  }

  @Test
  void testShotFiredFromAiming() throws Exception {
    tracker.intakeStarted();
    Thread.sleep(5);
    tracker.intakeComplete();
    Thread.sleep(5);
    tracker.indexComplete();
    assertEquals(CycleTracker.CyclePhase.AIMING, tracker.getCurrentPhase());
    Thread.sleep(5);

    // Fire directly from AIMING (skipping aimComplete -> SHOOTING)
    tracker.shotFired();
    assertEquals(CycleTracker.CyclePhase.RECOVERING, tracker.getCurrentPhase());

    tracker.recoveryComplete();
    assertEquals(CycleTracker.CyclePhase.IDLE, tracker.getCurrentPhase());
    assertEquals(1, tracker.getCompletedCycles());
  }

  @Test
  void testReset() throws Exception {
    runCompleteCycle();
    assertEquals(1, tracker.getCompletedCycles());

    tracker.reset();

    assertEquals(CycleTracker.CyclePhase.IDLE, tracker.getCurrentPhase());
    assertEquals(0, tracker.getCompletedCycles());
    assertEquals(0, tracker.getFailedCycles());
    assertEquals(0, tracker.getLastTotalCycleTimeMs(), 0.001);

    double bestMs = getField(tracker, "bestCycleTimeMs");
    assertEquals(Double.MAX_VALUE, bestMs, 0.001);
  }

  @Test
  void testSuccessRate() throws Exception {
    // 2 complete cycles
    runCompleteCycle();
    runCompleteCycle();

    // 1 aborted cycle
    tracker.intakeStarted();
    tracker.cycleAborted("test abort");

    assertEquals(2, tracker.getCompletedCycles());
    assertEquals(1, tracker.getFailedCycles());

    // Success rate = 2 / (2+1) * 100 = 66.67%
    // Read from internal calculation: completedCycles / (completed + failed) * 100
    double expectedRate = (2.0 / 3.0) * 100.0;
    double actualRate =
        (double) tracker.getCompletedCycles()
            / (tracker.getCompletedCycles() + tracker.getFailedCycles())
            * 100.0;
    assertEquals(expectedRate, actualRate, 0.01);
  }

  @Test
  void testLogDoesNotThrow() throws Exception {
    // Log in IDLE
    assertDoesNotThrow(() -> tracker.log());

    // Log mid-cycle
    tracker.intakeStarted();
    assertDoesNotThrow(() -> tracker.log());

    tracker.intakeComplete();
    assertDoesNotThrow(() -> tracker.log());

    tracker.indexComplete();
    assertDoesNotThrow(() -> tracker.log());

    tracker.aimComplete();
    assertDoesNotThrow(() -> tracker.log());

    tracker.shotFired();
    assertDoesNotThrow(() -> tracker.log());

    tracker.recoveryComplete();
    assertDoesNotThrow(() -> tracker.log());
  }

  /** Run a complete scoring cycle with small delays between phases */
  private void runCompleteCycle() throws Exception {
    tracker.intakeStarted();
    Thread.sleep(5);
    tracker.intakeComplete();
    Thread.sleep(5);
    tracker.indexComplete();
    Thread.sleep(5);
    tracker.aimComplete();
    Thread.sleep(5);
    tracker.shotFired();
    Thread.sleep(5);
    tracker.recoveryComplete();
  }
}
