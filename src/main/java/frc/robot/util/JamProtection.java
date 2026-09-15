package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;

/**
 * Reusable 3-layer jam detection and auto-reverse state machine for motor subsystems.
 *
 * <p>Layer 1: Startup ignore (suppress false triggers from inrush current). Layer 2: Sustained jam
 * confirmation (current + velocity debounce). Layer 3: Reverse-and-retry with attempt limiting.
 *
 * <p>Call {@link #update(double, double, boolean)} from the subsystem's periodic(). The state
 * machine is observe-only; telemetry reads the state for logging but never overrides motors.
 */
public class JamProtection {

  public enum State {
    /** Normal operation, monitoring for jam conditions. */
    MONITORING,
    /** Jam criteria met, waiting for debounce confirmation. */
    JAM_CONFIRMING,
    /** Jam confirmed, motor reversed to clear obstruction. */
    REVERSING,
    /** Brief pause after reverse before resuming forward. */
    COOLDOWN,
    /** Max attempts exceeded, motor disabled until manual reset. */
    DISABLED
  }

  private final String name;
  private final double jamCurrentAmps;
  private final double jamVelocityRPM;
  private final double startupIgnoreSec;
  private final double jamConfirmSec;
  private final double reverseTimeSec;
  private final double cooldownSec;
  private final double reversePower;
  private final int maxAttempts;
  private final DoubleSupplier clock;

  private State state = State.MONITORING;
  private double stateStartTime = 0;
  private double motorStartTime = 0;
  private boolean motorWasRunning = false;
  private int reverseAttempts = 0;

  /**
   * @param name subsystem name for logging
   * @param jamCurrentAmps current threshold to consider jammed
   * @param jamVelocityRPM velocity must be below this to consider jammed
   * @param startupIgnoreSec ignore jam signals for this long after motor starts
   * @param jamConfirmSec jam must persist this long to confirm
   * @param reverseTimeSec how long to reverse the motor
   * @param cooldownSec pause after reverse before resuming
   * @param reversePower motor power during reverse (negative value, e.g. -0.3)
   * @param maxAttempts disable motor after this many failed reverse attempts
   */
  public JamProtection(
      String name,
      double jamCurrentAmps,
      double jamVelocityRPM,
      double startupIgnoreSec,
      double jamConfirmSec,
      double reverseTimeSec,
      double cooldownSec,
      double reversePower,
      int maxAttempts) {
    this(
        name,
        jamCurrentAmps,
        jamVelocityRPM,
        startupIgnoreSec,
        jamConfirmSec,
        reverseTimeSec,
        cooldownSec,
        reversePower,
        maxAttempts,
        Timer::getFPGATimestamp);
  }

  /** Package-private constructor with injectable clock for testing. */
  JamProtection(
      String name,
      double jamCurrentAmps,
      double jamVelocityRPM,
      double startupIgnoreSec,
      double jamConfirmSec,
      double reverseTimeSec,
      double cooldownSec,
      double reversePower,
      int maxAttempts,
      DoubleSupplier clock) {
    this.name = name;
    this.jamCurrentAmps = jamCurrentAmps;
    this.jamVelocityRPM = jamVelocityRPM;
    this.startupIgnoreSec = startupIgnoreSec;
    this.jamConfirmSec = jamConfirmSec;
    this.reverseTimeSec = reverseTimeSec;
    this.cooldownSec = cooldownSec;
    this.reversePower = reversePower;
    this.maxAttempts = maxAttempts;
    this.clock = clock;
  }

  /**
   * Update the jam protection state machine. Call once per periodic cycle.
   *
   * @param currentAmps motor output current
   * @param velocityRPM motor velocity
   * @param running true if the motor is commanded to run
   * @return the current state for telemetry observation
   */
  public State update(double currentAmps, double velocityRPM, boolean running) {
    double now = clock.getAsDouble();

    // Detect motor start rising edge for startup ignore
    if (running && !motorWasRunning) {
      motorStartTime = now;
    }
    motorWasRunning = running;

    // If motor is not running, reset to monitoring (except DISABLED)
    if (!running && state != State.DISABLED) {
      state = State.MONITORING;
      return state;
    }

    switch (state) {
      case MONITORING:
        // Layer 1: startup ignore
        if (running && (now - motorStartTime) < startupIgnoreSec) {
          break;
        }
        // Layer 2: check jam criteria
        if (running && currentAmps > jamCurrentAmps && Math.abs(velocityRPM) < jamVelocityRPM) {
          state = State.JAM_CONFIRMING;
          stateStartTime = now;
        }
        break;

      case JAM_CONFIRMING:
        // If jam criteria no longer met, go back to monitoring
        if (!running || currentAmps <= jamCurrentAmps || Math.abs(velocityRPM) >= jamVelocityRPM) {
          state = State.MONITORING;
          break;
        }
        // Layer 2: sustained jam confirmed after debounce
        if ((now - stateStartTime) >= jamConfirmSec) {
          // Layer 3: reverse to clear
          if (reverseAttempts >= maxAttempts) {
            state = State.DISABLED;
            stateStartTime = now;
          } else {
            state = State.REVERSING;
            stateStartTime = now;
            reverseAttempts++;
          }
        }
        break;

      case REVERSING:
        // Hold reverse for the configured duration
        if ((now - stateStartTime) >= reverseTimeSec) {
          state = State.COOLDOWN;
          stateStartTime = now;
        }
        break;

      case COOLDOWN:
        // Brief pause before allowing forward again
        if ((now - stateStartTime) >= cooldownSec) {
          state = State.MONITORING;
          // Reset motorStartTime so startup ignore applies to the retry
          motorStartTime = now;
        }
        break;

      case DISABLED:
        // Stay disabled until manual reset
        break;
    }

    return state;
  }

  /** Reset jam protection (e.g., driver presses a button). Clears attempts and re-enables. */
  public void reset() {
    state = State.MONITORING;
    reverseAttempts = 0;
  }

  public State getState() {
    return state;
  }

  public int getReverseAttempts() {
    return reverseAttempts;
  }

  public boolean isDisabled() {
    return state == State.DISABLED;
  }

  public boolean isIntervening() {
    return state == State.REVERSING || state == State.COOLDOWN || state == State.DISABLED;
  }

  public String getName() {
    return name;
  }
}
