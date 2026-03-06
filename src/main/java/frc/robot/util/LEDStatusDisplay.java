package frc.robot.util;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.BatteryThresholds;
import frc.robot.Constants.LEDConstants;
import frc.robot.telemetry.TelemetryManager;

public class LEDStatusDisplay {
  private static LEDStatusDisplay instance;

  public enum LEDState {
    DISABLED,
    IDLE,
    MATCH_OVER,
    VISION_LOCKED,
    AUTO_RUNNING,
    WARNING,
    SHOOTER_SPINUP,
    AIM_PROGRESS,
    READY_TO_SHOOT,
    CRITICAL_ALERT
  }

  // Green instead of white bc Kfir said white wasn't visible enough on the field
  private static final int BLUE_R = 0, BLUE_G = 100, BLUE_B = 255;
  private static final int ORANGE_R = 255, ORANGE_G = 100, ORANGE_B = 0;
  private static final int GREEN_R = 0, GREEN_G = 255, GREEN_B = 0;
  private static final int RED_R = 255, RED_G = 0, RED_B = 0;

  // R203-M says flashing > 5 Hz is illegal and seizures can trigger at 3+ Hz,
  // so everything here stays under 1.5 Hz and nothing ever goes full black.
  private static final double MIN_BREATHE_PERIOD = 0.667; // 1.5 Hz max breathe rate
  private static final double BREATHE_FLOOR = 0.15; // never go fully dark (max 6.7:1 contrast)
  private static final double CHASE_DIM_FACTOR = 0.15; // dim background for "off" segments
  private static final int CHASE_SEGMENT_SIZE = 5; // LEDs per chase group

  // If battery voltage flickers right at the CRITICAL_V threshold, the state would
  // toggle green/red at 50 Hz without this hold. 0.5s minimum locks it to 1 Hz max.
  private static final double MIN_STATE_HOLD_SEC = 0.5;

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final boolean hardwareAvailable;
  private final int length;

  private LEDState currentState = LEDState.DISABLED;
  private LEDState previousState = LEDState.DISABLED;
  private int stateChangeCount = 0;
  private double lastStateChangeTime = 0;

  // Match-over latch
  private boolean matchEndDetected = false;
  private boolean prevEnabled = false;
  private double prevMatchTime = -1;

  // Rainbow pattern for auto
  private final LEDPattern rainbowPattern;

  // Tracks where we are in the aim breathe cycle so changing the period
  // doesn't make the brightness jump to a random spot.
  private double aimPhaseAccumulator = 0;
  private double aimLastUpdateTime = 0;

  private static final TunableNumber brightness = new TunableNumber("LED/brightness", 0.8);

  // --- Test state override (slider 0-11 from Elastic, FMS-locked) ---
  private static final LEDState[] TEST_STATES = LEDState.values();

  private static final String[] TEST_DESCRIPTIONS = {
    "0: Normal (no override)",
    "1: DISABLED - Dim green",
    "2: IDLE - Solid green",
    "3: MATCH_OVER - Green breathing",
    "4: VISION_LOCKED - Blue breathing",
    "5: AUTO_RUNNING - Rainbow scroll",
    "6: WARNING - Orange chase",
    "7: SHOOTER_SPINUP - Blue progress bar",
    "8: AIM_PROGRESS - Blue pulse (speed varies)",
    "9: READY_TO_SHOOT - Solid blue",
    "10: CRITICAL_ALERT - Red/orange chase"
  };

  private static final TunableNumber testState = new TunableNumber("LED/TestState", 0);
  private int lastTestStateValue = 0;

  // All the inputs we need to figure out which LED state to show
  record StateSnapshot(
      boolean enabled,
      boolean autonomous,
      boolean teleop,
      double matchTime,
      double batteryVoltage,
      boolean brownout,
      boolean readyToShoot,
      boolean progressiveAimActive,
      double progressiveAimError,
      boolean shooterSpinningUp,
      double shooterAtSpeedPercent,
      boolean jamDetected,
      boolean anyStalled,
      boolean batteryWarning,
      boolean canError,
      boolean visionLocked,
      boolean lowConfidence) {}

  private LEDStatusDisplay() {
    length = LEDConstants.STRIP_LENGTH;
    buffer = new AddressableLEDBuffer(length);
    rainbowPattern = LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Percent.per(Second).of(50));

    AddressableLED tempLed = null;
    boolean tempAvailable = false;
    try {
      tempLed = new AddressableLED(LEDConstants.PWM_PORT);
      tempLed.setLength(length);
      tempLed.start();
      tempAvailable = true;
    } catch (Throwable t) {
      tempAvailable = false;
    }
    led = tempLed;
    hardwareAvailable = tempAvailable;

    SmartDashboard.putStringArray("LED/TestMenu", TEST_DESCRIPTIONS);
  }

  public static LEDStatusDisplay getInstance() {
    if (instance == null) {
      instance = new LEDStatusDisplay();
    }
    return instance;
  }

  public void update() {
    if (!hardwareAvailable) return;

    StateSnapshot snapshot = buildSnapshot();
    updateMatchOverLatch(snapshot);
    LEDState resolved = resolveState(snapshot);

    // Test override from Elastic slider (FMS-locked)
    if (!DriverStation.isFMSAttached()) {
      int testVal = (int) testState.get();
      if (testVal >= 1 && testVal <= TEST_STATES.length) {
        resolved = TEST_STATES[testVal - 1];
        // Fake snapshot so the animated ones actually show up in test mode
        if (resolved == LEDState.SHOOTER_SPINUP || resolved == LEDState.AIM_PROGRESS) {
          snapshot =
              new StateSnapshot(
                  true, false, true, 120, 12.5, false, false, true, 5.0, true, 65.0, false, false,
                  false, false, false, false);
        }
      }
      lastTestStateValue = testVal;
    }

    double now = Timer.getFPGATimestamp();
    if (resolved != currentState && (now - lastStateChangeTime) >= MIN_STATE_HOLD_SEC) {
      previousState = currentState;
      currentState = resolved;
      stateChangeCount++;
      lastStateChangeTime = now;
    }

    renderState(currentState, snapshot);
    led.setData(buffer);
  }

  private StateSnapshot buildSnapshot() {
    TelemetryManager tm = TelemetryManager.getInstance();
    boolean enabled = DriverStation.isEnabled();
    boolean autonomous = DriverStation.isAutonomous();
    boolean teleop = DriverStation.isTeleop();
    double matchTime = DriverStation.getMatchTime();
    double batteryVoltage = tm.getBatteryVoltage();
    boolean brownout = tm.isBrownoutRisk();

    boolean readyToShoot = tm.isReadyToShoot();
    boolean progressiveAimActive = false;
    double progressiveAimError = -1;
    try {
      DriverFeedback df = DriverFeedback.getInstance();
      progressiveAimActive = df.isProgressiveAimActive();
      progressiveAimError = df.getProgressiveAimError();
    } catch (Throwable t) {
    }

    boolean shooterSpinningUp = tm.isShooterSpinningUp();
    double shooterAtSpeedPercent = tm.getShooterAtSpeedPercent();
    boolean jamDetected =
        tm.isIndexerJamDetected() || tm.isIntakeJamDetected() || tm.isAnyJamIntervening();
    boolean anyStalled =
        tm.isShooterStalled()
            || tm.isIndexerStalled()
            || tm.isIntakeStalled()
            || tm.isAgitatorStalled();
    boolean batteryWarning = batteryVoltage < BatteryThresholds.WARNING_V;
    boolean canError = !tm.isAllCANConnected();
    boolean visionLocked = tm.isLockedOnTarget();
    boolean lowConfidence = ChannelCoordinator.getInstance().isLowConfidence();

    return new StateSnapshot(
        enabled,
        autonomous,
        teleop,
        matchTime,
        batteryVoltage,
        brownout,
        readyToShoot,
        progressiveAimActive,
        progressiveAimError,
        shooterSpinningUp,
        shooterAtSpeedPercent,
        jamDetected,
        anyStalled,
        batteryWarning,
        canError,
        visionLocked,
        lowConfidence);
  }

  private void updateMatchOverLatch(StateSnapshot s) {
    // Reset latch on re-enable
    if (s.enabled() && !prevEnabled) {
      matchEndDetected = false;
      prevMatchTime = -1;
    }

    // Latch when match time hits 0 while enabled in teleop
    if (s.enabled()
        && s.teleop()
        && s.matchTime() >= 0
        && s.matchTime() <= 0.1
        && prevMatchTime > 0.1) {
      matchEndDetected = true;
    }

    prevEnabled = s.enabled();
    if (s.matchTime() >= 0) {
      prevMatchTime = s.matchTime();
    }
  }

  // Package-private for testing
  LEDState resolveState(StateSnapshot s) {
    if (!s.enabled()) return LEDState.DISABLED;

    // Priority 10: Critical
    if (s.batteryVoltage() < BatteryThresholds.CRITICAL_V || s.brownout()) {
      return LEDState.CRITICAL_ALERT;
    }

    // Priority 9: Ready to shoot
    if (s.readyToShoot()) return LEDState.READY_TO_SHOOT;

    // Priority 8: Aim progress
    if (s.progressiveAimActive() && s.progressiveAimError() >= 0) return LEDState.AIM_PROGRESS;

    // Priority 7: Shooter spinup
    if (s.shooterSpinningUp()) return LEDState.SHOOTER_SPINUP;

    // Priority 6: Warning (jam, stall, low battery, CAN error, low vision confidence)
    if (s.jamDetected() || s.anyStalled() || s.batteryWarning() || s.canError()
        || s.lowConfidence()) {
      return LEDState.WARNING;
    }

    // Priority 5: Auto running (rainbow)
    if (s.autonomous()) return LEDState.AUTO_RUNNING;

    // Priority 4: Vision locked
    if (s.visionLocked()) return LEDState.VISION_LOCKED;

    // Priority 3: Match over (latched)
    if (matchEndDetected) return LEDState.MATCH_OVER;

    // Priority 2: Idle (default)
    return LEDState.IDLE;
  }

  private void renderState(LEDState state, StateSnapshot s) {
    double bright = brightness.get();
    double now = Timer.getFPGATimestamp();

    switch (state) {
      case DISABLED ->
          fillSolid(
              scale(GREEN_R, LEDConstants.DIM_DISABLED * bright),
              scale(GREEN_G, LEDConstants.DIM_DISABLED * bright),
              scale(GREEN_B, LEDConstants.DIM_DISABLED * bright));

      case IDLE ->
          fillSolid(scale(GREEN_R, bright), scale(GREEN_G, bright), scale(GREEN_B, bright));

      case MATCH_OVER -> {
        double phase = breathePhase(now, 3.0);
        fillSolid(
            scale(GREEN_R, phase * bright),
            scale(GREEN_G, phase * bright),
            scale(GREEN_B, phase * bright));
      }

      case VISION_LOCKED -> {
        double phase = breathePhase(now, 2.0);
        fillSolid(
            scale(BLUE_R, phase * bright),
            scale(BLUE_G, phase * bright),
            scale(BLUE_B, phase * bright));
      }

      case AUTO_RUNNING -> rainbowPattern.applyTo(buffer);

      case WARNING -> {
        // Orange dots slide along the strip, 1 Hz per LED, dim background.
        renderChase(now, 1.0, ORANGE_R, ORANGE_G, ORANGE_B, bright);
      }

      case SHOOTER_SPINUP -> {
        double pct = Math.max(0, Math.min(100, s.shooterAtSpeedPercent())) / 100.0;
        int litCount = (int) (length * pct);
        for (int i = 0; i < length; i++) {
          if (i < litCount) {
            buffer.setRGB(i, scale(BLUE_R, bright), scale(BLUE_G, bright), scale(BLUE_B, bright));
          } else {
            // Dim blue shows the "unfilled" portion of the bar
            buffer.setRGB(
                i,
                scale(BLUE_R, CHASE_DIM_FACTOR * bright),
                scale(BLUE_G, CHASE_DIM_FACTOR * bright),
                scale(BLUE_B, CHASE_DIM_FACTOR * bright));
          }
        }
      }

      case AIM_PROGRESS -> {
        double error = Math.max(0, s.progressiveAimError());
        // Closer aim = faster pulse. Clamped so it can't go above 1.5 Hz.
        double period = Math.max(MIN_BREATHE_PERIOD, 0.5 + (error / 10.0) * 2.0);
        // Integrate dt/period so changing period doesn't jump the brightness.
        double dt = now - aimLastUpdateTime;
        if (dt > 0 && dt < 0.5) {
          aimPhaseAccumulator += dt / period;
        }
        aimLastUpdateTime = now;
        double raw = (Math.sin(2 * Math.PI * aimPhaseAccumulator) + 1.0) / 2.0;
        double phase = BREATHE_FLOOR + raw * (1.0 - BREATHE_FLOOR);
        fillSolid(
            scale(BLUE_R, phase * bright),
            scale(BLUE_G, phase * bright),
            scale(BLUE_B, phase * bright));
      }

      case READY_TO_SHOOT ->
          fillSolid(scale(BLUE_R, bright), scale(BLUE_G, bright), scale(BLUE_B, bright));

      case CRITICAL_ALERT -> {
        // Red and orange bands that slide along the strip at 1 Hz, no LED goes dark.
        // The old 20 Hz strobe broke R203-M and could trigger seizures.
        renderDualChase(now, 0.5, RED_R, RED_G, RED_B, ORANGE_R, ORANGE_G, ORANGE_B, bright);
      }
    }
  }

  /** Slides a bright chunk along the strip with the rest dimmed. */
  private void renderChase(double now, double period, int r, int g, int b, double bright) {
    int totalSegments = (length + CHASE_SEGMENT_SIZE - 1) / CHASE_SEGMENT_SIZE;
    int activeSegment = (int) ((now / period) * totalSegments) % totalSegments;

    for (int i = 0; i < length; i++) {
      int seg = i / CHASE_SEGMENT_SIZE;
      if (seg == activeSegment || seg == (activeSegment + 1) % totalSegments) {
        buffer.setRGB(i, scale(r, bright), scale(g, bright), scale(b, bright));
      } else {
        // Dim instead of black so nothing ever fully turns off
        buffer.setRGB(
            i,
            scale(r, CHASE_DIM_FACTOR * bright),
            scale(g, CHASE_DIM_FACTOR * bright),
            scale(b, CHASE_DIM_FACTOR * bright));
      }
    }
  }

  /** Two colors in alternating chunks that scroll together like a barber pole. */
  private void renderDualChase(
      double now, double period,
      int r1, int g1, int b1,
      int r2, int g2, int b2,
      double bright) {
    // Shift the pattern over time so it looks like it's moving
    int offset = (int) (now / period * CHASE_SEGMENT_SIZE) % (CHASE_SEGMENT_SIZE * 2);

    for (int i = 0; i < length; i++) {
      int pos = (i + offset) % (CHASE_SEGMENT_SIZE * 2);
      if (pos < CHASE_SEGMENT_SIZE) {
        buffer.setRGB(i, scale(r1, bright), scale(g1, bright), scale(b1, bright));
      } else {
        buffer.setRGB(i, scale(r2, bright), scale(g2, bright), scale(b2, bright));
      }
    }
  }

  private void fillSolid(int r, int g, int b) {
    for (int i = 0; i < length; i++) {
      buffer.setRGB(i, r, g, b);
    }
  }

  private static int scale(int value, double factor) {
    return (int) Math.max(0, Math.min(255, value * factor));
  }

  private static double breathePhase(double time, double period) {
    double raw = (Math.sin(2 * Math.PI * time / period) + 1.0) / 2.0;
    // Floor so breathing never goes fully dark
    return BREATHE_FLOOR + raw * (1.0 - BREATHE_FLOOR);
  }

  // --- Accessors for telemetry ---

  public String getCurrentStateName() {
    return currentState.name();
  }

  public boolean isHardwareAvailable() {
    return hardwareAvailable;
  }

  public double getCurrentBrightness() {
    return brightness.get();
  }

  public int getStateChangeCount() {
    return stateChangeCount;
  }

  public String getStateDescription() {
    int testVal = lastTestStateValue;
    if (testVal >= 0 && testVal < TEST_DESCRIPTIONS.length) {
      return TEST_DESCRIPTIONS[testVal];
    }
    return TEST_DESCRIPTIONS[0];
  }
}
