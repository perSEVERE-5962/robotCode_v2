package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.TunableNumber;

public class Shooter extends Actuator {
  private static Shooter instance;

  private SparkMax motor;
  private RelativeEncoder motorEncoder;
  private SparkMaxConfig motorConfig;

  private double targetRPM = 0;

  // --- Recovery state tracking ---
  // When a ball passes through the flywheel, RPM drops sharply. Normal PID
  // gains recover slowly because they're tuned for smooth steady-state holding.
  // By switching to an aggressive PID slot during that recovery window, the
  // SparkMax's 1kHz onboard loop can push the motor much harder and cut
  // recovery time roughly in half, so we're ready for the next ball faster.
  private boolean pidActive = false;
  private boolean wasAtSpeed = false;
  private boolean recovering = false;

  // Tunable PID values (slot 0: normal steady-state hold)
  private static final TunableNumber kP = new TunableNumber("Shooter/kP", ShooterConstants.P);
  private static final TunableNumber kI = new TunableNumber("Shooter/kI", ShooterConstants.I);
  private static final TunableNumber kD = new TunableNumber("Shooter/kD", ShooterConstants.D);
  private static final TunableNumber kF = new TunableNumber("Shooter/FF", ShooterConstants.FF);

  // Tunable PID values (slot 1: aggressive post-shot recovery)
  private static final TunableNumber recoveryKP =
      new TunableNumber("Shooter/Recovery/kP", ShooterConstants.RECOVERY_P);
  private static final TunableNumber recoveryKI =
      new TunableNumber("Shooter/Recovery/kI", ShooterConstants.RECOVERY_I);
  private static final TunableNumber recoveryKD =
      new TunableNumber("Shooter/Recovery/kD", ShooterConstants.RECOVERY_D);
  private static final TunableNumber recoveryKF =
      new TunableNumber("Shooter/Recovery/FF", ShooterConstants.RECOVERY_FF);

  // Tunable setpoints and thresholds
  private static final TunableNumber targetRPMTunable =
      new TunableNumber("Shooter/TargetRPM", ShooterConstants.TARGET_RPM);
  private static final TunableNumber toleranceRPM =
      new TunableNumber("Shooter/ToleranceRPM", ShooterConstants.SPEED_TOLERANCE_RPM);
  private static final TunableNumber shotDropRPM =
      new TunableNumber("Shooter/ShotDropRPM", ShooterConstants.SHOT_DETECTION_DROP_RPM);

  private Shooter() {
    super(
        Constants.CANDeviceIDs.kShooterID,
        ShooterConstants.P,
        ShooterConstants.I,
        ShooterConstants.D,
        ShooterConstants.MinOutput,
        ShooterConstants.MaxOutput,
        ShooterConstants.FF,
        ShooterConstants.Iz,
        0,
        0,
        false,
        false,
        false);
    motor = getMotor();

    motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    motorConfig.smartCurrentLimit(40);

    // Set up slot 1 with aggressive gains for recovery. Slot 0 was already
    // configured by the Actuator constructor with the normal gentle gains.
    // We keep them separate so the motor can switch behavior instantly
    // without reconfiguring gains on every transition.
    motorConfig.closedLoop
        .pid(
            ShooterConstants.RECOVERY_P,
            ShooterConstants.RECOVERY_I,
            ShooterConstants.RECOVERY_D,
            ClosedLoopSlot.kSlot1)
        .outputRange(ShooterConstants.MinOutput, ShooterConstants.MaxOutput, ClosedLoopSlot.kSlot1);
    motorConfig.closedLoop.feedForward.kV(
        12.0 * ShooterConstants.RECOVERY_FF, ClosedLoopSlot.kSlot1);

    motorEncoder = motor.getEncoder();

    motor.configure(
        motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getVelocityRPM() {
    return motorEncoder.getVelocity() * ShooterConstants.VELOCITY_CONVERSION;
  }

  public boolean isAtSpeed() {
    if (targetRPM == 0) return false;
    return Math.abs(targetRPM - getVelocityRPM()) < toleranceRPM.get();
  }

  @Override
  public void periodic() {
    // Hot-reload normal gains (slot 0)
    TunableNumber.ifChanged(
        () -> updatePID(kP.get(), kI.get(), kD.get(), kF.get()), kP, kI, kD, kF);

    // Hot-reload recovery gains (slot 1)
    TunableNumber.ifChanged(
        () ->
            updateRecoveryPID(
                recoveryKP.get(), recoveryKI.get(), recoveryKD.get(), recoveryKF.get()),
        recoveryKP,
        recoveryKI,
        recoveryKD,
        recoveryKF);

    updateRecoveryState();
  }

  /**
   * Tracks whether the flywheel is in post-shot recovery so we know which PID
   * slot to use. The logic is: once we reach target speed, watch for a sudden
   * RPM drop (ball passing through). When that happens, flag recovery mode.
   * When RPM comes back up, clear it.
   */
  private void updateRecoveryState() {
    if (!pidActive || targetRPM == 0) {
      recovering = false;
      wasAtSpeed = false;
      return;
    }

    double velocity = getVelocityRPM();
    double dropThreshold = shotDropRPM.get();
    double tolerance = toleranceRPM.get();

    if (!wasAtSpeed) {
      // Still spinning up for the first time, wait until we actually reach target
      if (Math.abs(targetRPM - velocity) < tolerance) {
        wasAtSpeed = true;
      }
    } else if (!recovering) {
      // Holding at speed, check for a shot-induced RPM dip
      if (velocity < (targetRPM - dropThreshold)) {
        recovering = true;
      }
    } else {
      // Recovering, check if we're back within tolerance
      if (Math.abs(targetRPM - velocity) < tolerance) {
        recovering = false;
        // wasAtSpeed stays true so we catch the next shot too
      }
    }
  }

  public void move(double speed) {
    motor.set(speed);
    // Switching to duty cycle mode means PID isn't driving anymore,
    // so reset all recovery tracking for a clean slate next spin-up.
    pidActive = false;
    wasAtSpeed = false;
    recovering = false;
  }

  @Override
  public void moveToVelocityWithPID(double rpm) {
    // If the target changed significantly (like switching shot distances),
    // that's a new setpoint ramp, not a post-shot recovery. Reset tracking
    // so we don't accidentally use aggressive gains for a normal spin-up.
    if (Math.abs(rpm - this.targetRPM) > toleranceRPM.get()) {
      wasAtSpeed = false;
      recovering = false;
    }

    this.targetRPM = rpm;
    this.pidActive = true;

    // Pick the right PID slot: aggressive during recovery, gentle otherwise
    ClosedLoopSlot slot = recovering ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot0;
    motor.getClosedLoopController().setSetpoint(rpm, SparkMax.ControlType.kVelocity, slot);
  }

  /**
   * Hot-reload recovery PID gains (slot 1) from dashboard without a full
   * motor reconfigure. Same pattern as updatePID() in the Actuator base class,
   * just targeting slot 1 instead of slot 0.
   */
  private void updateRecoveryPID(double kP, double kI, double kD, double kF) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot1);
    config.closedLoop.feedForward.kV(12.0 * kF, ClosedLoopSlot.kSlot1);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public double getMotorVelocity() {
    return getVelocityRPM();
  }

  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  // --- Hardware accessors ---

  public double getTargetRPM() {
    return targetRPM;
  }

  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  public double getOutputCurrent() {
    return motor.getOutputCurrent();
  }

  public double getBusVoltage() {
    return motor.getBusVoltage();
  }

  // --- Recovery state accessors ---

  /** True when the flywheel is actively recovering from a ball pass-through. */
  public boolean isRecovering() {
    return recovering;
  }

  /** True when PID velocity control is actively driving the motor. */
  public boolean isPidActive() {
    return pidActive;
  }

  // --- Tunable accessors ---

  public double getTunableTargetRPM() {
    return targetRPMTunable.get();
  }

  public double getToleranceRPM() {
    return toleranceRPM.get();
  }

  public double getShotDropThreshold() {
    return shotDropRPM.get();
  }

  public double getTunableKP() {
    return kP.get();
  }

  public double getTunableKI() {
    return kI.get();
  }

  public double getTunableKD() {
    return kD.get();
  }

  public double getTunableFF() {
    return kF.get();
  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }
}
