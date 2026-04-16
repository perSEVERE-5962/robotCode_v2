package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.HeadingLockConstants;

/**
 * Heading controller for SOTM and hub arc drive. PID + optional feedforward + optional lookahead.
 * Each layer has a TunableNumber kill switch. Falls back to P-only if both PID and FF are off. Each
 * command should own its own instance and call reset() in initialize().
 */
public class HeadingController {

  // PID gains
  private final TunableNumber kP = new TunableNumber("HeadingLock/kP", HeadingLockConstants.P);
  private final TunableNumber kI = new TunableNumber("HeadingLock/kI", HeadingLockConstants.I);
  private final TunableNumber kD = new TunableNumber("HeadingLock/kD", HeadingLockConstants.D);

  // Feedforward gains
  private final TunableNumber kS =
      new TunableNumber("HeadingLock/FF/kS", HeadingLockConstants.FF_KS);
  private final TunableNumber kV =
      new TunableNumber("HeadingLock/FF/kV", HeadingLockConstants.FF_KV);
  private final TunableNumber kA =
      new TunableNumber("HeadingLock/FF/kA", HeadingLockConstants.FF_KA);

  // Lookahead
  private final TunableNumber kLookaheadSec =
      new TunableNumber("HeadingLock/lookaheadSec", HeadingLockConstants.LOOKAHEAD_SEC);

  // FF distance scaling
  private final TunableNumber kFFMinDist =
      new TunableNumber("HeadingLock/FF/minDistM", HeadingLockConstants.FF_MIN_DIST_M);
  private final TunableNumber kFFMaxDist =
      new TunableNumber("HeadingLock/FF/maxDistM", HeadingLockConstants.FF_MAX_DIST_M);

  // Per-layer kill switches
  private final TunableNumber kEnablePID = new TunableNumber("HeadingLock/enablePID", 1.0);
  private final TunableNumber kEnableFF = new TunableNumber("HeadingLock/enableFF", 0.0);
  private final TunableNumber kEnableLookahead =
      new TunableNumber("HeadingLock/enableLookahead", 0.0);

  private final PIDController pid;
  private final LinearFilter omegaFilter = LinearFilter.movingAverage(5);

  private double filteredOmega = 0;
  private double previousFilteredOmega = 0;
  private double lastPIDOutput = 0;
  private double lastFFOutput = 0;
  private double lastTotalOutput = 0;

  // Hysteresis: once at setpoint, widen tolerance 4x so brief oscillations
  // near the boundary don't flicker ReadyToShoot.
  private static final double HYSTERESIS_FACTOR = 4.0;
  private boolean stickyAtSetpoint = false;

  public HeadingController() {
    pid = new PIDController(kP.get(), kI.get(), kD.get());
    pid.enableContinuousInput(-Math.PI, Math.PI);
    pid.setTolerance(Math.toRadians(1.0));
  }

  /** Returns omega (rad/s) to drive heading toward target. */
  public double calculate(
      Rotation2d currentHeading,
      Rotation2d targetHeading,
      double desiredOmegaRadPerSec,
      double distanceToTargetM,
      double maxOmegaRadPerSec) {

    // Update PID gains if changed on dashboard
    TunableNumber.ifChanged(() -> pid.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);

    // Filter desired omega to reduce noise (5-sample = 100ms at 50Hz)
    filteredOmega = omegaFilter.calculate(desiredOmegaRadPerSec);

    boolean pidEnabled = kEnablePID.get() > 0.5;
    boolean ffEnabled = kEnableFF.get() > 0.5;
    boolean lookaheadEnabled = kEnableLookahead.get() > 0.5;

    double headingSpeed = 0;

    // Layer 1: PID
    if (pidEnabled) {
      double target = targetHeading.getRadians();

      // Layer 3: Lookahead (shifts the target the PID aims at)
      if (lookaheadEnabled && kLookaheadSec.get() > 0) {
        double dt = kLookaheadSec.get();
        target = target + filteredOmega * dt;
      }

      lastPIDOutput = pid.calculate(currentHeading.getRadians(), target);
      headingSpeed = lastPIDOutput;
    } else {
      lastPIDOutput = 0;
    }

    // base omega so PID only corrects error, not the full tracking rate
    headingSpeed += filteredOmega;

    // Layer 2: Feedforward
    if (ffEnabled) {
      double distScale = computeDistanceScale(distanceToTargetM);

      // Angular acceleration from filtered omega (finite difference at 50Hz)
      double alpha = (filteredOmega - previousFilteredOmega) / 0.02;

      lastFFOutput =
          (kS.get() * Math.signum(filteredOmega) + kV.get() * filteredOmega + kA.get() * alpha)
              * distScale;

      headingSpeed += lastFFOutput;
    } else {
      lastFFOutput = 0;
    }

    // Fallback: legacy P-only when both PID and FF disabled
    if (!pidEnabled && !ffEnabled) {
      double error =
          MathUtil.angleModulus(targetHeading.getRadians() - currentHeading.getRadians());
      headingSpeed = error * 2.0 + desiredOmegaRadPerSec;
      lastPIDOutput = 0;
      lastFFOutput = 0;
    }

    previousFilteredOmega = filteredOmega;

    lastTotalOutput = MathUtil.clamp(headingSpeed, -maxOmegaRadPerSec, maxOmegaRadPerSec);
    return lastTotalOutput;
  }

  /** FF scales 0..1 by distance so it doesn't overcorrect up close. */
  double computeDistanceScale(double distance) {
    double minD = kFFMinDist.get();
    double maxD = kFFMaxDist.get();
    if (distance <= minD) return 0.0;
    if (distance >= maxD) return 1.0;
    return (distance - minD) / (maxD - minD);
  }

  /** Reset all state. Call in command initialize(). */
  public void reset() {
    pid.reset();
    omegaFilter.reset();
    filteredOmega = 0;
    previousFilteredOmega = 0;
    lastPIDOutput = 0;
    lastFFOutput = 0;
    lastTotalOutput = 0;
    stickyAtSetpoint = false;
  }

  // Telemetry accessors
  public double getFilteredOmega() {
    return filteredOmega;
  }

  public double getPositionError() {
    return pid.getError();
  }

  public double getPIDOutput() {
    return lastPIDOutput;
  }

  public double getFFOutput() {
    return lastFFOutput;
  }

  public double getTotalOutput() {
    return lastTotalOutput;
  }

  /** Sticky setpoint with 4x hysteresis band so it doesn't flicker near the edge. */
  public boolean atSetpoint() {
    double errorRad = Math.abs(pid.getError());
    double toleranceRad = Math.toRadians(1.0);

    if (!stickyAtSetpoint) {
      if (errorRad <= toleranceRad) {
        stickyAtSetpoint = true;
      }
    } else {
      if (errorRad > toleranceRad * HYSTERESIS_FACTOR) {
        stickyAtSetpoint = false;
      }
    }
    return stickyAtSetpoint;
  }

  public boolean isPIDEnabled() {
    return kEnablePID.get() > 0.5;
  }

  public boolean isFFEnabled() {
    return kEnableFF.get() > 0.5;
  }

  public boolean isLookaheadEnabled() {
    return kEnableLookahead.get() > 0.5;
  }
}
