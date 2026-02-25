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

  // Colorblind-safe palette
  private static final int BLUE_R = 0, BLUE_G = 100, BLUE_B = 255;
  private static final int ORANGE_R = 255, ORANGE_G = 100, ORANGE_B = 0;
  private static final int WHITE_R = 255, WHITE_G = 255, WHITE_B = 255;
  private static final int RED_R = 255, RED_G = 0, RED_B = 0;

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final boolean hardwareAvailable;
  private final int length;

  private LEDState currentState = LEDState.DISABLED;
  private LEDState previousState = LEDState.DISABLED;
  private int stateChangeCount = 0;

  // Match-over latch
  private boolean matchEndDetected = false;
  private boolean prevEnabled = false;
  private double prevMatchTime = -1;

  // Rainbow pattern for auto
  private final LEDPattern rainbowPattern;

  private static final TunableNumber brightness = new TunableNumber("LED/brightness", 0.8);

  // --- Test state override (slider 0-11 from Elastic, FMS-locked) ---
  private static final LEDState[] TEST_STATES = LEDState.values();

  private static final String[] TEST_DESCRIPTIONS = {
    "0: Normal (no override)",
    "1: DISABLED - Dim white",
    "2: IDLE - Solid white",
    "3: MATCH_OVER - White breathing",
    "4: VISION_LOCKED - Blue breathing",
    "5: AUTO_RUNNING - Rainbow scroll",
    "6: WARNING - Orange breathing",
    "7: SHOOTER_SPINUP - Blue progress bar",
    "8: AIM_PROGRESS - Blue pulse (speed varies)",
    "9: READY_TO_SHOOT - Solid blue",
    "10: CRITICAL_ALERT - Red/orange flash"
  };

  private static final TunableNumber testState = new TunableNumber("LED/TestState", 0);
  private int lastTestStateValue = 0;

  // Snapshot of all inputs needed for state resolution
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
        // Fake snapshot values so animated states render visibly
        if (resolved == LEDState.SHOOTER_SPINUP || resolved == LEDState.AIM_PROGRESS) {
          snapshot =
              new StateSnapshot(
                  true, false, true, 120, 12.5, false, false, true, 5.0, true, 65.0, false, false,
                  false, false, false, false);
        }
      }
      lastTestStateValue = testVal;
    }

    if (resolved != currentState) {
      previousState = currentState;
      currentState = resolved;
      stateChangeCount++;
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
              scale(WHITE_R, LEDConstants.DIM_DISABLED * bright),
              scale(WHITE_G, LEDConstants.DIM_DISABLED * bright),
              scale(WHITE_B, LEDConstants.DIM_DISABLED * bright));

      case IDLE ->
          fillSolid(scale(WHITE_R, bright), scale(WHITE_G, bright), scale(WHITE_B, bright));

      case MATCH_OVER -> {
        double phase = breathePhase(now, 3.0);
        fillSolid(
            scale(WHITE_R, phase * bright),
            scale(WHITE_G, phase * bright),
            scale(WHITE_B, phase * bright));
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
        double phase = breathePhase(now, 1.5);
        fillSolid(
            scale(ORANGE_R, phase * bright),
            scale(ORANGE_G, phase * bright),
            scale(ORANGE_B, phase * bright));
      }

      case SHOOTER_SPINUP -> {
        double pct = Math.max(0, Math.min(100, s.shooterAtSpeedPercent())) / 100.0;
        int litCount = (int) (length * pct);
        for (int i = 0; i < length; i++) {
          if (i < litCount) {
            buffer.setRGB(i, scale(BLUE_R, bright), scale(BLUE_G, bright), scale(BLUE_B, bright));
          } else {
            buffer.setRGB(i, 0, 0, 0);
          }
        }
      }

      case AIM_PROGRESS -> {
        double error = Math.max(0, s.progressiveAimError());
        double period = 0.3 + (error / 10.0) * 2.2;
        double phase = breathePhase(now, period);
        fillSolid(
            scale(BLUE_R, phase * bright),
            scale(BLUE_G, phase * bright),
            scale(BLUE_B, phase * bright));
      }

      case READY_TO_SHOOT ->
          fillSolid(scale(BLUE_R, bright), scale(BLUE_G, bright), scale(BLUE_B, bright));

      case CRITICAL_ALERT -> {
        boolean useRed = ((int) (now / 0.05)) % 2 == 0;
        if (useRed) {
          fillSolid(scale(RED_R, bright), scale(RED_G, bright), scale(RED_B, bright));
        } else {
          fillSolid(scale(ORANGE_R, bright), scale(ORANGE_G, bright), scale(ORANGE_B, bright));
        }
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
    return (Math.sin(2 * Math.PI * time / period) + 1.0) / 2.0;
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
