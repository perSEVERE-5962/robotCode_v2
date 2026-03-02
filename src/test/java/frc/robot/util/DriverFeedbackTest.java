package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.telemetry.SafeLog;
import frc.robot.util.DriverFeedback.HapticPattern;
import frc.robot.util.DriverFeedback.HapticTarget;
import frc.robot.util.DriverFeedback.Priority;
import frc.robot.util.DriverFeedback.Step;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriverFeedbackTest {

  private static final int DRIVER_PORT = 0;
  private static final int COPILOT_PORT = 1;
  private DriverFeedback feedback;
  private GenericHID driverController;
  private GenericHIDSim driverSim;
  private GenericHID copilotController;
  private GenericHIDSim copilotSim;

  @BeforeEach
  void setUp() throws Exception {
    HAL.initialize(500, 0);
    setEnabled(false);

    Field f = DriverFeedback.class.getDeclaredField("instance");
    f.setAccessible(true);
    f.set(null, null);

    driverController = new GenericHID(DRIVER_PORT);
    driverSim = new GenericHIDSim(driverController);
    copilotController = new GenericHID(COPILOT_PORT);
    copilotSim = new GenericHIDSim(copilotController);

    // Mark both controllers as physically connected so isConnected() returns true.
    // WPILib checks axisCount > 0 || buttonCount > 0 || povCount > 0.
    driverSim.setAxisCount(6);
    driverSim.setButtonCount(10);
    copilotSim.setAxisCount(6);
    copilotSim.setButtonCount(10);
    DriverStationSim.notifyNewData();

    feedback = DriverFeedback.getInstance();
    feedback.initialize(driverController, copilotController);
  }

  @AfterEach
  void tearDown() throws Exception {
    feedback.stopAll();
    Field f = DriverFeedback.class.getDeclaredField("instance");
    f.setAccessible(true);
    f.set(null, null);
    SafeLog.logAndReset();
  }

  @Test
  void testPlayPatternSetsMotorValues() {
    HapticPattern pattern =
        new HapticPattern("TEST", new Step[] {new Step(0.5, 0.7, 1.0)}, Priority.HIGH,
            HapticTarget.BOTH);
    feedback.playPattern(pattern);

    assertEquals("TEST", feedback.getActivePatternName());
    assertEquals("HIGH", feedback.getActivePriority());
    assertEquals(0.5, feedback.getLeftMotor(), 0.01);
    assertEquals(0.7, feedback.getRightMotor(), 0.01);
  }

  @Test
  void testHigherPriorityInterruptsLower() {
    HapticPattern low =
        new HapticPattern("LOW_PAT", new Step[] {new Step(0.1, 0.1, 5.0)}, Priority.HIGH,
            HapticTarget.BOTH);
    HapticPattern high =
        new HapticPattern("HIGH_PAT", new Step[] {new Step(0.9, 0.9, 5.0)}, Priority.CRITICAL,
            HapticTarget.BOTH);

    feedback.playPattern(low);
    assertEquals("LOW_PAT", feedback.getActivePatternName());

    feedback.playPattern(high);
    assertEquals("HIGH_PAT", feedback.getActivePatternName());
    assertEquals(0.9, feedback.getLeftMotor(), 0.01);
  }

  @Test
  void testLowerPriorityDoesNotInterrupt() {
    HapticPattern high =
        new HapticPattern("HIGH_PAT", new Step[] {new Step(0.9, 0.9, 5.0)}, Priority.CRITICAL,
            HapticTarget.BOTH);
    HapticPattern low =
        new HapticPattern("LOW_PAT", new Step[] {new Step(0.1, 0.1, 5.0)}, Priority.HIGH,
            HapticTarget.BOTH);

    feedback.playPattern(high);
    feedback.playPattern(low);
    assertEquals("HIGH_PAT", feedback.getActivePatternName());
  }

  @Test
  void testSamePriorityReplaces() {
    HapticPattern first =
        new HapticPattern("FIRST", new Step[] {new Step(0.3, 0.3, 5.0)}, Priority.HIGH,
            HapticTarget.BOTH);
    HapticPattern second =
        new HapticPattern("SECOND", new Step[] {new Step(0.6, 0.6, 5.0)}, Priority.HIGH,
            HapticTarget.BOTH);

    feedback.playPattern(first);
    feedback.playPattern(second);
    assertEquals("SECOND", feedback.getActivePatternName());
  }

  @Test
  void testProgressiveAimQuadraticMapping() {
    feedback.setProgressiveAim(0);
    assertTrue(feedback.isProgressiveAimActive());
    assertEquals(0, feedback.getProgressiveAimError(), 0.001);
  }

  @Test
  void testProgressiveAimClear() {
    feedback.setProgressiveAim(3.0);
    feedback.clearProgressiveAim();
    assertFalse(feedback.isProgressiveAimActive());
    assertEquals(-1, feedback.getProgressiveAimError(), 0.001);
  }

  @Test
  void testNullControllerDoesNotThrow() throws Exception {
    Field f = DriverFeedback.class.getDeclaredField("instance");
    f.setAccessible(true);
    f.set(null, null);

    DriverFeedback nullFeedback = DriverFeedback.getInstance();
    assertDoesNotThrow(() -> nullFeedback.update());
  }

  @Test
  void testStopAllClearsEverything() {
    feedback.playPattern(DriverFeedback.READY_TO_SHOOT);
    feedback.setProgressiveAim(3.0);

    feedback.stopAll();

    assertEquals("none", feedback.getActivePatternName());
    assertEquals(0, feedback.getLeftMotor(), 0.001);
    assertEquals(0, feedback.getRightMotor(), 0.001);
    assertFalse(feedback.isProgressiveAimActive());
    assertEquals(-1, feedback.getProgressiveAimError(), 0.001);
  }

  @Test
  void testReadyToShootPatternValues() {
    feedback.playPattern(DriverFeedback.READY_TO_SHOOT);
    assertEquals(0, feedback.getLeftMotor(), 0.01);
    assertEquals(0.3, feedback.getRightMotor(), 0.01);
  }

  @Test
  void testRumbleRoutesToCopilotForScoringPattern() {
    setEnabled(true);
    // READY_TO_SHOOT targets COPILOT, so copilot controller should rumble
    feedback.playPattern(DriverFeedback.READY_TO_SHOOT);
    feedback.update();

    double copilotLeft = copilotSim.getRumble(GenericHID.RumbleType.kLeftRumble);
    double copilotRight = copilotSim.getRumble(GenericHID.RumbleType.kRightRumble);
    assertEquals(0, copilotLeft, 0.02);
    assertTrue(copilotRight > 0, "Copilot right rumble should be active for READY_TO_SHOOT");

    // Driver should NOT rumble for COPILOT-targeted pattern
    double driverLeft = driverSim.getRumble(GenericHID.RumbleType.kLeftRumble);
    double driverRight = driverSim.getRumble(GenericHID.RumbleType.kRightRumble);
    assertEquals(0, driverLeft, 0.02);
    assertEquals(0, driverRight, 0.02);
  }

  @Test
  void testRumbleRoutesToBothForCriticalPattern() {
    setEnabled(true);
    // TELEOP_START targets BOTH, so both controllers should rumble
    feedback.playPattern(DriverFeedback.TELEOP_START);
    feedback.update();

    double driverLeft = driverSim.getRumble(GenericHID.RumbleType.kLeftRumble);
    double driverRight = driverSim.getRumble(GenericHID.RumbleType.kRightRumble);
    assertTrue(driverLeft > 0, "Driver left rumble should be active for TELEOP_START");
    assertTrue(driverRight > 0, "Driver right rumble should be active for TELEOP_START");

    double copilotLeft = copilotSim.getRumble(GenericHID.RumbleType.kLeftRumble);
    double copilotRight = copilotSim.getRumble(GenericHID.RumbleType.kRightRumble);
    assertTrue(copilotLeft > 0, "Copilot left rumble should be active for TELEOP_START");
    assertTrue(copilotRight > 0, "Copilot right rumble should be active for TELEOP_START");
  }

  @Test
  void testDisabledModeClearsActiveRumble() {
    setEnabled(true);
    feedback.playPattern(DriverFeedback.TELEOP_START);
    feedback.update();

    assertTrue(driverSim.getRumble(GenericHID.RumbleType.kLeftRumble) > 0);
    assertTrue(driverSim.getRumble(GenericHID.RumbleType.kRightRumble) > 0);

    setEnabled(false);
    feedback.update();

    assertEquals(0, driverSim.getRumble(GenericHID.RumbleType.kLeftRumble), 0.02);
    assertEquals(0, driverSim.getRumble(GenericHID.RumbleType.kRightRumble), 0.02);
    assertEquals(0, copilotSim.getRumble(GenericHID.RumbleType.kLeftRumble), 0.02);
    assertEquals(0, copilotSim.getRumble(GenericHID.RumbleType.kRightRumble), 0.02);
  }

  @Test
  void testProgressiveAimAutoClearsWhenStale() {
    setEnabled(true);
    feedback.setProgressiveAim(2.0);
    feedback.update();

    assertTrue(feedback.isProgressiveAimActive());
    assertTrue(copilotSim.getRumble(GenericHID.RumbleType.kRightRumble) > 0);

    SimHooks.stepTiming(0.30); // > stale timeout (250ms)
    feedback.update();

    assertFalse(feedback.isProgressiveAimActive());
    assertEquals(0, copilotSim.getRumble(GenericHID.RumbleType.kLeftRumble), 0.02);
    assertEquals(0, copilotSim.getRumble(GenericHID.RumbleType.kRightRumble), 0.02);
  }

  // ==================== ACTIVATE / DEACTIVATE / NEVER-ACTIVATE TRIPLES ====================
  // Pattern 7 from FMEA: "If it turns on, prove it turns off"

  @Test
  void testTeleopStartActivatesAndExpires() {
    setEnabled(true);
    feedback.playPattern(DriverFeedback.TELEOP_START);
    assertEquals("TELEOP_START", feedback.getActivePatternName());
    assertTrue(feedback.getLeftMotor() > 0, "Left motor should be active");
    assertTrue(feedback.getRightMotor() > 0, "Right motor should be active");

    // TELEOP_START is 1 step of 0.3s. Advance past it.
    SimHooks.stepTiming(0.35);
    feedback.update();

    assertEquals("none", feedback.getActivePatternName());
    assertEquals(0, feedback.getLeftMotor(), 0.01);
    assertEquals(0, feedback.getRightMotor(), 0.01);
  }

  @Test
  void testEndgameWarningActivatesAndExpires() {
    setEnabled(true);
    feedback.playPattern(DriverFeedback.ENDGAME_WARNING);
    assertEquals("ENDGAME_WARNING", feedback.getActivePatternName());

    // ENDGAME_WARNING: 4 steps (0.15 + 0.05 + 0.15 + 0.15 = 0.50s total)
    // Must call update() for each step transition
    for (int i = 0; i < 4; i++) {
      SimHooks.stepTiming(0.20);
      feedback.update();
    }

    assertEquals("none", feedback.getActivePatternName());
    assertEquals(0, feedback.getLeftMotor(), 0.01);
    assertEquals(0, feedback.getRightMotor(), 0.01);
  }

  @Test
  void testReadyToShootActivatesAndExpires() {
    setEnabled(true);
    feedback.playPattern(DriverFeedback.READY_TO_SHOOT);
    assertEquals("READY_TO_SHOOT", feedback.getActivePatternName());
    assertEquals(0, feedback.getLeftMotor(), 0.01);
    assertTrue(feedback.getRightMotor() > 0, "Right motor should be active");

    // READY_TO_SHOOT: 1 step of 0.25s
    SimHooks.stepTiming(0.30);
    feedback.update();

    assertEquals("none", feedback.getActivePatternName());
    assertEquals(0, feedback.getRightMotor(), 0.01);
  }

  @Test
  void testHubActivatedActivatesAndExpires() {
    setEnabled(true);
    feedback.playPattern(DriverFeedback.HUB_ACTIVATED);
    assertEquals("HUB_ACTIVATED", feedback.getActivePatternName());

    // HUB_ACTIVATED: 3 steps (0.15 + 0.05 + 0.15 = 0.35s total)
    for (int i = 0; i < 3; i++) {
      SimHooks.stepTiming(0.20);
      feedback.update();
    }

    assertEquals("none", feedback.getActivePatternName());
    assertEquals(0, feedback.getLeftMotor(), 0.01);
    assertEquals(0, feedback.getRightMotor(), 0.01);
  }

  @Test
  void testHubDeactivatedActivatesAndExpires() {
    setEnabled(true);
    feedback.playPattern(DriverFeedback.HUB_DEACTIVATED);
    assertEquals("HUB_DEACTIVATED", feedback.getActivePatternName());
    assertTrue(feedback.getLeftMotor() > 0, "Left motor should be active");

    // HUB_DEACTIVATED: 1 step of 0.3s
    SimHooks.stepTiming(0.35);
    feedback.update();

    assertEquals("none", feedback.getActivePatternName());
    assertEquals(0, feedback.getLeftMotor(), 0.01);
  }

  @Test
  void testHubShiftWarningActivatesAndExpires() {
    setEnabled(true);
    feedback.playPattern(DriverFeedback.HUB_SHIFT_WARNING);
    assertEquals("HUB_SHIFT_WARNING", feedback.getActivePatternName());

    // HUB_SHIFT_WARNING: 5 steps (0.1+0.05+0.1+0.05+0.1 = 0.40s total)
    for (int i = 0; i < 5; i++) {
      SimHooks.stepTiming(0.15);
      feedback.update();
    }

    assertEquals("none", feedback.getActivePatternName());
    assertEquals(0, feedback.getLeftMotor(), 0.01);
    assertEquals(0, feedback.getRightMotor(), 0.01);
  }

  @Test
  void testJamDetectedActivatesAndExpires() {
    setEnabled(true);
    feedback.playPattern(DriverFeedback.JAM_DETECTED);
    assertEquals("JAM_DETECTED", feedback.getActivePatternName());

    // JAM_DETECTED: 5 steps (0.2+0.1+0.2+0.1+0.2 = 0.80s total)
    for (int i = 0; i < 5; i++) {
      SimHooks.stepTiming(0.25);
      feedback.update();
    }

    assertEquals("none", feedback.getActivePatternName());
    assertEquals(0, feedback.getLeftMotor(), 0.01);
    assertEquals(0, feedback.getRightMotor(), 0.01);
  }

  @Test
  void testProgressiveAimNeverActivatesWithoutSet() {
    setEnabled(true);
    // Never call setProgressiveAim - progressive aim should not be active
    feedback.update();

    assertFalse(feedback.isProgressiveAimActive());
    assertEquals(-1, feedback.getProgressiveAimError(), 0.001);
  }

  @Test
  void testPatternNeverActivatesWhenDisabled() {
    setEnabled(false);
    feedback.playPattern(DriverFeedback.TELEOP_START);
    feedback.update();

    // Pattern is queued but rumble should be zero because disabled
    assertEquals(0, driverSim.getRumble(GenericHID.RumbleType.kLeftRumble), 0.02);
    assertEquals(0, driverSim.getRumble(GenericHID.RumbleType.kRightRumble), 0.02);
    assertEquals(0, copilotSim.getRumble(GenericHID.RumbleType.kLeftRumble), 0.02);
    assertEquals(0, copilotSim.getRumble(GenericHID.RumbleType.kRightRumble), 0.02);
  }

  @Test
  void testAllPatternsExpireToZeroRumble() {
    setEnabled(true);

    // Play every pattern and verify each one expires to zero rumble
    HapticPattern[] allPatterns = {
      DriverFeedback.TELEOP_START, DriverFeedback.ENDGAME_WARNING,
      DriverFeedback.READY_TO_SHOOT, DriverFeedback.HUB_ACTIVATED,
      DriverFeedback.HUB_DEACTIVATED, DriverFeedback.HUB_SHIFT_WARNING,
      DriverFeedback.JAM_DETECTED
    };

    for (HapticPattern pattern : allPatterns) {
      feedback.playPattern(pattern);
      // Advance through all steps (max 5 steps per pattern, 0.5s each > any step)
      for (int i = 0; i < 6; i++) {
        SimHooks.stepTiming(0.5);
        feedback.update();
      }

      assertEquals("none", feedback.getActivePatternName(),
          pattern.name() + " should have expired");
      assertEquals(0, driverSim.getRumble(GenericHID.RumbleType.kLeftRumble), 0.02,
          pattern.name() + " driver left should be zero after expiry");
      assertEquals(0, driverSim.getRumble(GenericHID.RumbleType.kRightRumble), 0.02,
          pattern.name() + " driver right should be zero after expiry");
      assertEquals(0, copilotSim.getRumble(GenericHID.RumbleType.kLeftRumble), 0.02,
          pattern.name() + " copilot left should be zero after expiry");
      assertEquals(0, copilotSim.getRumble(GenericHID.RumbleType.kRightRumble), 0.02,
          pattern.name() + " copilot right should be zero after expiry");
    }
  }

  private static void setEnabled(boolean enabled) {
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }
}
