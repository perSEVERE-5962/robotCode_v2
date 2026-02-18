package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.telemetry.TelemetryManager;

public class DriverFeedback {
  private static DriverFeedback instance;

  // --- Inner types ---

  public record Step(double left, double right, double durationSec) {}

  public record HapticPattern(String name, Step[] steps, Priority priority) {}

  public enum Priority {
    LOW,
    MEDIUM,
    HIGH,
    CRITICAL
  }

  // --- Pattern library (5 discrete patterns, v3.0 human-factors-refined) ---

  // CRITICAL: match phase
  public static final HapticPattern TELEOP_START =
      new HapticPattern("TELEOP_START", new Step[] {new Step(1.0, 1.0, 0.3)}, Priority.CRITICAL);

  public static final HapticPattern ENDGAME_WARNING =
      new HapticPattern(
          "ENDGAME_WARNING",
          new Step[] {
            new Step(0.6, 0.6, 0.15),
            new Step(0, 0, 0.05),
            new Step(0.6, 0.6, 0.15),
            new Step(0, 0, 0.15)
          },
          Priority.CRITICAL);

  // HIGH: scoring and hub state
  public static final HapticPattern READY_TO_SHOOT =
      new HapticPattern("READY_TO_SHOOT", new Step[] {new Step(0, 0.3, 0.25)}, Priority.HIGH);

  public static final HapticPattern HUB_ACTIVATED =
      new HapticPattern(
          "HUB_ACTIVATED",
          new Step[] {new Step(0, 0.5, 0.15), new Step(0, 0, 0.05), new Step(0, 0.5, 0.15)},
          Priority.HIGH);

  public static final HapticPattern HUB_DEACTIVATED =
      new HapticPattern("HUB_DEACTIVATED", new Step[] {new Step(0.5, 0, 0.3)}, Priority.HIGH);

  // --- Test pattern table (indexed 1-5 from Elastic slider) ---
  private static final HapticPattern[] TEST_PATTERNS = {
    TELEOP_START, ENDGAME_WARNING, READY_TO_SHOOT, HUB_ACTIVATED, HUB_DEACTIVATED
  };

  private static final String[] TEST_DESCRIPTIONS = {
    "1: TELEOP_START - Full buzz, teleop begins",
    "2: ENDGAME_WARNING - 2 pulses, 30s warning",
    "3: READY_TO_SHOOT - Right tap, ready to fire",
    "4: HUB_ACTIVATED - 2 right pings, hub live",
    "5: HUB_DEACTIVATED - Left thump, hub off"
  };

  // Elastic slider: set 1-5 to play that pattern, 0 = off
  private static final TunableNumber testPattern =
      new TunableNumber("DriverFeedback/TestPattern", 0);
  private int lastTestPatternValue = 0;

  // --- Playback engine state ---
  private HapticPattern activePattern = null;
  private int stepIndex = 0;
  private double stepStartTime = 0;
  private double currentLeft = 0;
  private double currentRight = 0;
  private int patternCount = 0;

  // --- Edge detection state ---
  private boolean prevReadyToShoot = false;
  private boolean prevHubActive = true; // default true (safe: no false trigger on startup)
  private boolean endgameWarningPlayed = false;
  private boolean prevEnabled = false;
  private boolean prevAutonomous = false;

  // --- Progressive aim ---
  private boolean progressiveAimActive = false;
  private double progressiveAimError = -1;

  // --- Accessibility ---
  private static final TunableNumber hapticScale =
      new TunableNumber("DriverFeedback/hapticScale", 1.0);

  // --- Controller ---
  private GenericHID controller = null;

  private DriverFeedback() {}

  public static DriverFeedback getInstance() {
    if (instance == null) {
      instance = new DriverFeedback();
    }
    return instance;
  }

  public void initialize(GenericHID controller) {
    this.controller = controller;
    SmartDashboard.putStringArray("DriverFeedback/TestMenu", TEST_DESCRIPTIONS);
  }

  public void update() {
    if (controller == null) return;

    // Test pattern trigger from Elastic slider (disabled at FMS)
    if (!DriverStation.isFMSAttached()) {
      int testVal = (int) testPattern.get();
      if (testVal != lastTestPatternValue && testVal >= 1 && testVal <= TEST_PATTERNS.length) {
        playPattern(TEST_PATTERNS[testVal - 1]);
      }
      lastTestPatternValue = testVal;
    }

    double now = Timer.getFPGATimestamp();

    // Read state from TelemetryManager (wrapped in try-catch)
    boolean readyToShoot = false;
    boolean hubActive = true;
    double matchTime = -1;
    boolean isEnabled = false;
    boolean isAutonomous = false;

    try {
      TelemetryManager tm = TelemetryManager.getInstance();
      readyToShoot = tm.isReadyToShoot();
      hubActive = tm.isHubActive();
      matchTime = DriverStation.getMatchTime();
      isEnabled = DriverStation.isEnabled();
      isAutonomous = DriverStation.isAutonomous();
    } catch (Throwable t) {
      // State read failed, use defaults
    }

    // Reset edge detection flags on enable
    if (isEnabled && !prevEnabled) {
      endgameWarningPlayed = false;
      prevHubActive = hubActive;
    }

    // --- Match phase events (CRITICAL) ---
    if (matchTime >= 0 && isEnabled) {
      if (!isAutonomous && prevAutonomous) {
        playPattern(TELEOP_START);
      }
      if (!isAutonomous && matchTime <= 30 && !endgameWarningPlayed) {
        playPattern(ENDGAME_WARNING);
        endgameWarningPlayed = true;
      }
    }

    // --- Hub shift events (HIGH) ---
    if (hubActive && !prevHubActive) {
      playPattern(HUB_ACTIVATED);
    }
    if (!hubActive && prevHubActive) {
      playPattern(HUB_DEACTIVATED);
    }

    // --- Scoring events (HIGH) ---
    if (readyToShoot && !prevReadyToShoot) {
      playPattern(READY_TO_SHOOT);
    }

    // Save edge detection state
    prevReadyToShoot = readyToShoot;
    prevHubActive = hubActive;
    prevEnabled = isEnabled;
    prevAutonomous = isAutonomous;

    // --- Pattern playback ---
    if (activePattern != null) {
      Step step = activePattern.steps()[stepIndex];
      double elapsed = now - stepStartTime;
      if (elapsed >= step.durationSec()) {
        stepIndex++;
        if (stepIndex >= activePattern.steps().length) {
          activePattern = null;
          stepIndex = 0;
          currentLeft = 0;
          currentRight = 0;
        } else {
          stepStartTime = now;
          Step next = activePattern.steps()[stepIndex];
          currentLeft = next.left();
          currentRight = next.right();
        }
      } else {
        currentLeft = step.left();
        currentRight = step.right();
      }
    }

    // --- Progressive aim (only when no pattern active) ---
    if (activePattern == null && progressiveAimActive && progressiveAimError >= 0) {
      if (progressiveAimError > 10.0) {
        currentLeft = 0;
        currentRight = 0;
      } else {
        double normalized = 1.0 - (progressiveAimError / 10.0);
        double intensity = normalized * normalized;
        currentLeft = intensity * 0.2;
        currentRight = intensity * 0.5;
      }
    } else if (activePattern == null && !progressiveAimActive) {
      currentLeft = 0;
      currentRight = 0;
    }

    // Apply scale and clamp
    double scale = hapticScale.get();
    double left = Math.max(0, Math.min(1, currentLeft * scale));
    double right = Math.max(0, Math.min(1, currentRight * scale));

    controller.setRumble(RumbleType.kLeftRumble, left);
    controller.setRumble(RumbleType.kRightRumble, right);
  }

  public void playPattern(HapticPattern pattern) {
    if (pattern == null) return;

    // Higher or equal priority replaces current
    if (activePattern != null
        && pattern.priority().ordinal() < activePattern.priority().ordinal()) {
      return;
    }

    activePattern = pattern;
    stepIndex = 0;
    stepStartTime = Timer.getFPGATimestamp();
    patternCount++;

    if (pattern.steps().length > 0) {
      currentLeft = pattern.steps()[0].left();
      currentRight = pattern.steps()[0].right();
    }
  }

  public void setProgressiveAim(double errorDeg) {
    progressiveAimActive = true;
    progressiveAimError = Math.abs(errorDeg);
  }

  public void clearProgressiveAim() {
    progressiveAimActive = false;
    progressiveAimError = -1;
  }

  public void stopAll() {
    activePattern = null;
    stepIndex = 0;
    currentLeft = 0;
    currentRight = 0;
    progressiveAimActive = false;
    progressiveAimError = -1;

    if (controller != null) {
      controller.setRumble(RumbleType.kLeftRumble, 0);
      controller.setRumble(RumbleType.kRightRumble, 0);
    }
  }

  // --- Accessors for telemetry ---

  public String getActivePatternName() {
    return activePattern != null ? activePattern.name() : "none";
  }

  public String getActivePriority() {
    return activePattern != null ? activePattern.priority().name() : "NONE";
  }

  public double getLeftMotor() {
    double scale = hapticScale.get();
    return Math.max(0, Math.min(1, currentLeft * scale));
  }

  public double getRightMotor() {
    double scale = hapticScale.get();
    return Math.max(0, Math.min(1, currentRight * scale));
  }

  public double getProgressiveAimError() {
    return progressiveAimError;
  }

  public int getPatternCount() {
    return patternCount;
  }

  public boolean isProgressiveAimActive() {
    return progressiveAimActive;
  }

  /** Human-readable description of the active pattern. */
  public String getPatternDescription() {
    if (activePattern == null) return "Idle";
    for (int i = 0; i < TEST_PATTERNS.length; i++) {
      if (TEST_PATTERNS[i] == activePattern) {
        return TEST_DESCRIPTIONS[i];
      }
    }
    return activePattern.name();
  }
}
