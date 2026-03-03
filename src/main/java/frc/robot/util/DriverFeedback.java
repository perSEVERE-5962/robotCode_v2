package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.telemetry.SafeLog;
import frc.robot.telemetry.TelemetryManager;

public class DriverFeedback {
  private static DriverFeedback instance;

  // --- Inner types ---

  public record Step(double left, double right, double durationSec) {}

  public enum HapticTarget {
    DRIVER,
    COPILOT,
    BOTH
  }

  public record HapticPattern(String name, Step[] steps, Priority priority, HapticTarget target) {}

  public enum Priority {
    LOW,
    MEDIUM,
    HIGH,
    CRITICAL
  }

  // --- Haptic patterns ---

  // CRITICAL: auto result at teleop start -> BOTH controllers
  // Both start with a strong buzz (teleop started!), then indicate win/loss.
  // Right follow-up = won auto (right side = scoring/positive in our pattern language)
  // Left follow-up = lost auto (left side = warning/alert)
  public static final HapticPattern AUTO_WON =
      new HapticPattern(
          "AUTO_WON",
          new Step[] {
            new Step(1.0, 1.0, 0.2),
            new Step(0, 0, 0.1),
            new Step(0, 0.5, 0.15),
            new Step(0, 0, 0.05),
            new Step(0, 0.5, 0.15)
          },
          Priority.CRITICAL,
          HapticTarget.BOTH);

  public static final HapticPattern AUTO_LOST =
      new HapticPattern(
          "AUTO_LOST",
          new Step[] {
            new Step(1.0, 1.0, 0.2),
            new Step(0, 0, 0.1),
            new Step(0.7, 0, 0.3)
          },
          Priority.CRITICAL,
          HapticTarget.BOTH);

  public static final HapticPattern ENDGAME_WARNING =
      new HapticPattern(
          "ENDGAME_WARNING",
          new Step[] {
            new Step(0.6, 0.6, 0.15),
            new Step(0, 0, 0.05),
            new Step(0.6, 0.6, 0.15),
            new Step(0, 0, 0.15)
          },
          Priority.CRITICAL,
          HapticTarget.BOTH);

  // HIGH: scoring and hub state -> COPILOT (copilot manages weapons)
  public static final HapticPattern READY_TO_SHOOT =
      new HapticPattern(
          "READY_TO_SHOOT",
          new Step[] {new Step(0, 0.3, 0.25)},
          Priority.HIGH,
          HapticTarget.COPILOT);

  public static final HapticPattern HUB_ACTIVATED =
      new HapticPattern(
          "HUB_ACTIVATED",
          new Step[] {new Step(0, 0.5, 0.15), new Step(0, 0, 0.05), new Step(0, 0.5, 0.15)},
          Priority.HIGH,
          HapticTarget.COPILOT);

  public static final HapticPattern HUB_DEACTIVATED =
      new HapticPattern(
          "HUB_DEACTIVATED",
          new Step[] {new Step(0.5, 0, 0.3)},
          Priority.HIGH,
          HapticTarget.COPILOT);

  // HIGH: jam protection intervening -> COPILOT
  public static final HapticPattern JAM_DETECTED =
      new HapticPattern(
          "JAM_DETECTED",
          new Step[] {
            new Step(0.8, 0.8, 0.2),
            new Step(0, 0, 0.1),
            new Step(0.8, 0.8, 0.2),
            new Step(0, 0, 0.1),
            new Step(0.8, 0.8, 0.2)
          },
          Priority.HIGH,
          HapticTarget.COPILOT);

  // CRITICAL: game data missing alert -> BOTH (repeats every 2s during transition)
  // FMS attached but no 'R'/'B' auto-winner message yet. Hub shift logic is guessing.
  public static final HapticPattern GAME_DATA_MISSING =
      new HapticPattern(
          "GAME_DATA_MISSING",
          new Step[] {
            new Step(1.0, 1.0, 0.2),
            new Step(0, 0, 0.1),
            new Step(1.0, 1.0, 0.2),
            new Step(0, 0, 0.1),
            new Step(1.0, 1.0, 0.2)
          },
          Priority.CRITICAL,
          HapticTarget.BOTH);

  // MEDIUM: predictive warning -> BOTH controllers
  public static final HapticPattern HUB_SHIFT_WARNING =
      new HapticPattern(
          "HUB_SHIFT_WARNING",
          new Step[] {
            new Step(0.3, 0.3, 0.1),
            new Step(0, 0, 0.05),
            new Step(0.3, 0.3, 0.1),
            new Step(0, 0, 0.05),
            new Step(0.3, 0.3, 0.1)
          },
          Priority.MEDIUM,
          HapticTarget.BOTH);

  // --- Test pattern table (indexed 1-9 from Elastic slider) ---
  private static final HapticPattern[] TEST_PATTERNS = {
    AUTO_WON, AUTO_LOST, ENDGAME_WARNING, READY_TO_SHOOT, HUB_ACTIVATED,
    HUB_DEACTIVATED, HUB_SHIFT_WARNING, JAM_DETECTED, GAME_DATA_MISSING
  };

  private static final String[] TEST_DESCRIPTIONS = {
    "1: AUTO_WON - Buzz + 2 right pings, we won auto",
    "2: AUTO_LOST - Buzz + left thump, we lost auto",
    "3: ENDGAME_WARNING - 2 pulses, 30s warning",
    "4: READY_TO_SHOOT - Right tap, ready to fire",
    "5: HUB_ACTIVATED - 2 right pings, hub live",
    "6: HUB_DEACTIVATED - Left thump, hub off",
    "7: HUB_SHIFT_WARNING - 3 quick taps, shift coming",
    "8: JAM_DETECTED - 3 strong pulses, jam auto-reverse",
    "9: GAME_DATA_MISSING - 3 strong, FMS data missing"
  };

  // Elastic slider: set 1-7 to play that pattern, 0 = off
  private static final TunableNumber testPattern =
      new TunableNumber("DriverFeedback/TestPattern", 0);
  private int lastTestPatternValue = 0;

  // --- Playback engine state ---
  private HapticPattern activePattern = null;
  private HapticTarget activeTarget = HapticTarget.BOTH;
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

  // --- Jam protection edge detection ---
  private boolean prevJamIntervening = false;
  private String lastJamSource = "none";

  // --- Progressive aim ---
  private boolean progressiveAimActive = false;
  private double progressiveAimError = -1;
  private static final double PROGRESSIVE_AIM_STALE_TIMEOUT_SEC = 0.25;
  private double lastProgressiveAimUpdateSec = -1;

  // --- Spin-up rumble (continuous background on DRIVER) ---
  private double spinUpPercent = 0;

  // --- Hub countdown warning ---
  private static final double HUB_SHIFT_WARN_THRESHOLD_SEC = 2.5;
  private boolean hubShiftWarningPlayed = false;
  private double prevTimeToNextShift = 0;

  // --- Game data missing alert (repeats during transition) ---
  // Transition period is ~10s from teleop start. If FMS is attached but hasn't
  // sent the 'R'/'B' auto-winner message, hub shift logic is running blind.
  private static final double GAME_DATA_TRANSITION_SEC = 10.0;
  private static final double GAME_DATA_ALERT_INTERVAL_SEC = 2.0;
  private double lastGameDataAlertTime = 0;
  private double teleopStartTime = -1;

  // --- Accessibility ---
  private static final TunableNumber hapticScale =
      new TunableNumber("DriverFeedback/hapticScale", 1.0);

  // --- Controllers ---
  private GenericHID controller = null;
  private GenericHID copilotController = null;

  private DriverFeedback() {}

  public static DriverFeedback getInstance() {
    if (instance == null) {
      instance = new DriverFeedback();
    }
    return instance;
  }

  public void initialize(GenericHID driver, GenericHID copilot) {
    this.controller = driver;
    this.copilotController = copilot;
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

    // Safety: progressive aim must be refreshed continuously by the command using it.
    // If updates stop, auto-clear so stale rumble cannot persist indefinitely.
    if (progressiveAimActive
        && lastProgressiveAimUpdateSec >= 0
        && (now - lastProgressiveAimUpdateSec) > PROGRESSIVE_AIM_STALE_TIMEOUT_SEC) {
      clearProgressiveAim();
    }

    // Read state from TelemetryManager (wrapped in try-catch)
    boolean readyToShoot = false;
    boolean hubActive = true;
    double timeToNextShift = 0;
    double matchTime = -1;
    boolean isEnabled = false;
    boolean isAutonomous = false;
    boolean jamIntervening = false;
    boolean wonAuto = false;

    try {
      TelemetryManager tm = TelemetryManager.getInstance();
      readyToShoot = tm.isReadyToShoot();
      hubActive = tm.isHubActive();
      timeToNextShift = tm.getTimeToNextShiftSec();
      spinUpPercent = tm.getFilteredSpinUpPercent();
      jamIntervening = tm.isAnyJamIntervening();
      wonAuto = tm.isWonAuto();
      matchTime = DriverStation.getMatchTime();
      isEnabled = DriverStation.isEnabled();
      isAutonomous = DriverStation.isAutonomous();
    } catch (Throwable t) {
      // State read failed, use defaults
    }

    // Reset edge detection flags on enable
    if (isEnabled && !prevEnabled) {
      endgameWarningPlayed = false;
      hubShiftWarningPlayed = false;
      prevHubActive = hubActive;
    }

    // --- No haptics while disabled ---
    if (!isEnabled) {
      // Keep edge state in sync so we don't get false triggers on re-enable
      prevReadyToShoot = readyToShoot;
      prevHubActive = hubActive;
      prevJamIntervening = jamIntervening;
      prevEnabled = isEnabled;
      prevAutonomous = isAutonomous;
      prevTimeToNextShift = timeToNextShift;
      // Zero any leftover rumble
      applyRumble(controller, 0, 0);
      applyRumble(copilotController, 0, 0);
      return;
    }

    // --- Match phase events (CRITICAL -> BOTH) ---
    // At auto-to-teleop transition, signal whether we won or lost auto.
    // FMS sends 'R' or 'B' via game-specific message; ScoringTelemetry parses it.
    // Won auto = our hub inactive first (collect/defend), lost = hub active (score NOW).
    if (matchTime >= 0 && isEnabled) {
      if (!isAutonomous && prevAutonomous) {
        playPattern(wonAuto ? AUTO_WON : AUTO_LOST);
        teleopStartTime = now;
      }
      if (!isAutonomous && matchTime <= 30 && !endgameWarningPlayed) {
        playPattern(ENDGAME_WARNING);
        endgameWarningPlayed = true;
      }
    }

    // --- Game data missing alert (CRITICAL -> BOTH, repeats every 2s) ---
    // FMS attached but no auto-winner data during transition period = hub logic is guessing
    if (isEnabled && !isAutonomous && teleopStartTime > 0) {
      double teleopElapsed = now - teleopStartTime;
      String gameMsg = DriverStation.getGameSpecificMessage();
      boolean gameDataMissing = DriverStation.isFMSAttached()
          && (gameMsg == null || gameMsg.isEmpty())
          && teleopElapsed < GAME_DATA_TRANSITION_SEC;
      if (gameDataMissing && (now - lastGameDataAlertTime) >= GAME_DATA_ALERT_INTERVAL_SEC) {
        playPattern(GAME_DATA_MISSING);
        lastGameDataAlertTime = now;
      }
      if (!gameDataMissing) {
        lastGameDataAlertTime = 0;
      }
    }

    // --- Hub shift events (HIGH -> COPILOT) ---
    if (hubActive && !prevHubActive) {
      playPattern(HUB_ACTIVATED);
    }
    if (!hubActive && prevHubActive) {
      playPattern(HUB_DEACTIVATED);
    }

    // --- Hub countdown warning (MEDIUM -> BOTH, edge-triggered per shift) ---
    if (timeToNextShift > 0
        && timeToNextShift <= HUB_SHIFT_WARN_THRESHOLD_SEC
        && prevTimeToNextShift > HUB_SHIFT_WARN_THRESHOLD_SEC
        && !hubShiftWarningPlayed) {
      playPattern(HUB_SHIFT_WARNING);
      hubShiftWarningPlayed = true;
    }
    // Reset flag when countdown resets (new shift window)
    if (timeToNextShift > HUB_SHIFT_WARN_THRESHOLD_SEC) {
      hubShiftWarningPlayed = false;
    }
    prevTimeToNextShift = timeToNextShift;

    // --- Scoring events (HIGH -> COPILOT) ---
    if (readyToShoot && !prevReadyToShoot) {
      playPattern(READY_TO_SHOOT);
    }

    // --- Jam protection events (HIGH -> COPILOT) ---
    if (jamIntervening && !prevJamIntervening) {
      playPattern(JAM_DETECTED);
      // Capture which subsystem triggered so pit crew can see it in telemetry
      try {
        lastJamSource = TelemetryManager.getInstance().getJamSource();
      } catch (Throwable t) {
        lastJamSource = "unknown";
      }
    } else if (!jamIntervening && prevJamIntervening) {
      lastJamSource = "none";
    }

    // Save edge detection state
    prevReadyToShoot = readyToShoot;
    prevHubActive = hubActive;
    prevJamIntervening = jamIntervening;
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
          activeTarget = HapticTarget.BOTH;
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

    // --- Progressive aim (only when no pattern active, routes to COPILOT) ---
    HapticTarget rumbleTarget = activeTarget;
    if (activePattern == null && progressiveAimActive && progressiveAimError >= 0) {
      rumbleTarget = HapticTarget.COPILOT;
      if (progressiveAimError > 10.0) {
        currentLeft = 0;
        currentRight = 0;
      } else {
        double normalized = 1.0 - (progressiveAimError / 10.0);
        double intensity = normalized * normalized;
        currentLeft = intensity * 0.2;
        currentRight = intensity * 0.5;
      }
    } else if (activePattern == null && spinUpPercent > 5 && spinUpPercent < 95) {
      // Spin-up rumble: driver feels shooter winding up as left-side rumble
      // In LOW confidence mode, amplify intensity (urgency: vision degraded while spinning up)
      rumbleTarget = HapticTarget.DRIVER;
      double normalized = spinUpPercent / 100.0;
      double maxIntensity = ChannelCoordinator.getInstance().isLowConfidence() ? 0.7 : 0.4;
      currentLeft = normalized * maxIntensity;
      currentRight = 0;
    } else if (activePattern == null && !progressiveAimActive) {
      currentLeft = 0;
      currentRight = 0;
    }

    // Apply scale and clamp
    double scale = hapticScale.get();
    double left = Math.max(0, Math.min(1, currentLeft * scale));
    double right = Math.max(0, Math.min(1, currentRight * scale));

    // Route rumble to appropriate controller(s)
    // Null check + physical connection check: WPILib creates the object even if no
    // controller is plugged in, so we need isConnected() to detect the real hardware.
    boolean hasCopilot = (copilotController != null && copilotController.isConnected());
    boolean driverRumble = (rumbleTarget == HapticTarget.DRIVER || rumbleTarget == HapticTarget.BOTH)
        || (rumbleTarget == HapticTarget.COPILOT && !hasCopilot);
    boolean copilotRumble =
        (rumbleTarget == HapticTarget.COPILOT || rumbleTarget == HapticTarget.BOTH);
    applyRumble(controller, driverRumble ? left : 0, driverRumble ? right : 0);
    applyRumble(copilotController, copilotRumble ? left : 0, copilotRumble ? right : 0);

    // Diagnostic signals so pit crew can verify copilot is actually connected
    SafeLog.put("DriverFeedback/CopilotConnected", hasCopilot);
    SafeLog.put("DriverFeedback/CopilotPort",
        copilotController != null ? copilotController.getPort() : -1);
    SafeLog.put("DriverFeedback/JamSource", lastJamSource);
  }

  private void applyRumble(GenericHID hid, double left, double right) {
    if (hid != null) {
      hid.setRumble(RumbleType.kLeftRumble, left);
      hid.setRumble(RumbleType.kRightRumble, right);
    }
  }

  public void playPattern(HapticPattern pattern) {
    if (pattern == null) return;

    // Higher or equal priority replaces current
    if (activePattern != null
        && pattern.priority().ordinal() < activePattern.priority().ordinal()) {
      return;
    }

    activePattern = pattern;
    activeTarget = pattern.target();
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
    lastProgressiveAimUpdateSec = Timer.getFPGATimestamp();
  }

  public void clearProgressiveAim() {
    progressiveAimActive = false;
    progressiveAimError = -1;
    lastProgressiveAimUpdateSec = -1;
  }

  public void stopAll() {
    activePattern = null;
    activeTarget = HapticTarget.BOTH;
    stepIndex = 0;
    currentLeft = 0;
    currentRight = 0;
    progressiveAimActive = false;
    progressiveAimError = -1;
    lastProgressiveAimUpdateSec = -1;
    spinUpPercent = 0;
    hubShiftWarningPlayed = false;
    prevTimeToNextShift = 0;
    lastGameDataAlertTime = 0;
    teleopStartTime = -1;

    applyRumble(controller, 0, 0);
    applyRumble(copilotController, 0, 0);
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

  public String getActiveTargetName() {
    return activeTarget.name();
  }

  public String getLastJamSource() {
    return lastJamSource;
  }

  public boolean isCopilotConnected() {
    return copilotController != null && copilotController.isConnected();
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
