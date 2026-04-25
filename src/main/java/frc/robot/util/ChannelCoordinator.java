package frc.robot.util;

import frc.robot.Constants;
import frc.robot.telemetry.TelemetryManager;

/**
 * Coordinates all driver feedback channels (haptic, LED, HUD) based on vision confidence. When
 * confidence drops below a threshold, all channels shift to emphasize the degraded state. When
 * confidence is high, channels operate normally.
 */
public class ChannelCoordinator {
  private static ChannelCoordinator instance;

  public enum ConfidenceMode {
    HIGH,
    LOW
  }

  private static final double LOW_THRESHOLD = 40.0;
  private static final double HIGH_THRESHOLD = 55.0; // hysteresis band

  private ConfidenceMode mode = ConfidenceMode.HIGH;
  private double confidence = 0;
  private boolean modeChanged = false;
  private ConfidenceMode previousMode = ConfidenceMode.HIGH;

  private ChannelCoordinator() {}

  public static ChannelCoordinator getInstance() {
    if (instance == null) {
      instance = new ChannelCoordinator();
    }
    return instance;
  }

  /** Call once per robotPeriodic(), after TelemetryManager.updateAll(). */
  public void update() {
    try {
      confidence = TelemetryManager.getInstance().getPoseConfidence();
    } catch (Throwable t) {
      confidence = 0;
    }

    previousMode = mode;

    // Hysteresis: drop to LOW at 40, recover to HIGH at 55
    if (mode == ConfidenceMode.HIGH && confidence <= LOW_THRESHOLD) {
      mode = ConfidenceMode.LOW;
    } else if (mode == ConfidenceMode.LOW && confidence >= HIGH_THRESHOLD) {
      mode = ConfidenceMode.HIGH;
    }

    modeChanged = (mode != previousMode);
  }

  public void log() {
    SafeLog.put("AMDA/Mode", mode.name());
    SafeLog.put("AMDA/Confidence", confidence);
    if (Constants.TUNING_MODE) {
      SafeLog.put("AMDA/ModeChanged", modeChanged);
    }
  }

  public ConfidenceMode getMode() {
    return mode;
  }

  public double getConfidence() {
    return confidence;
  }

  public boolean isModeChanged() {
    return modeChanged;
  }

  public boolean isLowConfidence() {
    return mode == ConfidenceMode.LOW;
  }
}
