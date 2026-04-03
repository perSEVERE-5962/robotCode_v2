package frc.robot.sim;

import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Shooter;
import frc.robot.telemetry.SafeLog;

/**
 * SparkSim wiring for motor subsystems. Call update() each simulationPeriodic(). First-order
 * dynamics bypass real PID gains (tuned for hardware, not sim).
 */
public class SimDeviceManager {
  private static final double NEO_FREE_SPEED_RPM = 5676.0;
  private static final double DT = 0.020;
  private static final double SPINUP_TAU = 0.100; // 100ms spin-up
  private static final double COASTDOWN_TAU = 0.300; // 300ms coast-down

  private SparkSim shooterSim;
  private SparkSim indexerSim;
  private SparkSim intakeSim;
  private SparkSim agitatorSim;
  private SparkSim intakeActuatorSim;
  private SparkSim hangerSim;
  private boolean initialized = false;

  // Per-motor RPM state for first-order dynamics
  private double shooterRPM = 0;
  private double indexerRPM = 0;
  private double intakeRPM = 0;
  private double agitatorRPM = 0;

  // Position motor state with sticky targets (same command-conflict fix as shooter)
  private double intakeActuatorPos = 0;
  private double hangerPos = 0;
  private static final double POSITION_TAU = 0.300; // 300ms approach
  private double lastActuatorTarget = 0;
  private int actuatorHoldCycles = 0;
  private double lastHangerTarget = 0;
  private int hangerHoldCycles = 0;

  // Sticky target: holds last nonzero target for a brief grace period,
  // avoiding brief 0s from command scheduler transitions
  private double lastShooterTarget = 0;
  private int targetHoldCycles = 0;
  private static final int TARGET_HOLD_COUNT = 10; // 200ms grace

  // Shot simulation: periodic RPM drops when indexing at speed
  private static final double SHOT_RPM_DROP = 350.0;
  private static final double SHOT_SIM_INTERVAL = 0.500; // one shot per 500ms
  private double lastShotSimTime = 0;
  private boolean shotFiredThisCycle = false;

  public void init() {
    try {
      Shooter shooter = Shooter.getInstance();
      Indexer indexer = Indexer.getInstance();
      IntakeRoller intake = IntakeRoller.getInstance();
      Agitator agitator = Agitator.getInstance();
      IntakePivot intakeActuator = IntakePivot.getInstance();
      Hanger hanger = Hanger.getInstance();
      if (shooter == null
          || indexer == null
          || intake == null
          || agitator == null
          || intakeActuator == null
          || hanger == null) {
        SafeLog.put("Sim/DeviceManager/InitError", "NullSubsystem");
        initialized = false;
        return;
      }
      shooterSim = new SparkSim(shooter.getMotor(), DCMotor.getNEO(1));
      indexerSim = new SparkSim(indexer.getMotor(), DCMotor.getNEO(1));
      intakeSim = new SparkSim(intake.getMotor(), DCMotor.getNEO(1));
      // agitatorSim = new SparkSim(agitator.getMotor(), DCMotor.getNEO(1));
      intakeActuatorSim = new SparkSim(intakeActuator.getMotor(), DCMotor.getNEO(1));
      hangerSim = new SparkSim(hanger.getMotor(), DCMotor.getNEO(1));
      initialized = true;
    } catch (RuntimeException e) {
      SafeLog.put("Sim/DeviceManager/InitError", e.getClass().getSimpleName());
      initialized = false;
    }
  }

  /** Call from Robot.simulationPeriodic() */
  public void update() {
    if (!initialized) return;
    try {
      // Shooter: sticky target rides through brief command transitions
      Shooter shooterInst = Shooter.getInstance();
      if (shooterInst == null) return;
      double rawTarget = shooterInst.getTargetRPM();
      if (rawTarget > 0) {
        lastShooterTarget = rawTarget;
        targetHoldCycles = TARGET_HOLD_COUNT;
      } else if (targetHoldCycles > 0) {
        targetHoldCycles--;
      } else {
        lastShooterTarget = 0;
      }
      double shooterTarget = lastShooterTarget;
      shooterRPM = updateMotorToTarget(shooterSim, shooterRPM, shooterTarget);

      double indexerTarget =
          (Math.abs(indexerSim.getAppliedOutput()) > 0.01)
              ? Constants.MotorConstants.DESIRED_INDEXER_RPM
              : 0;
      indexerRPM = updateMotorToTarget(indexerSim, indexerRPM, indexerTarget);

      double intakeOutput = intakeSim.getAppliedOutput();
      double intakeTarget = intakeOutput * NEO_FREE_SPEED_RPM;
      intakeRPM = updateMotorToTarget(intakeSim, intakeRPM, intakeTarget);

      double agitatorOutput = agitatorSim.getAppliedOutput();
      double agitatorTarget = agitatorOutput * NEO_FREE_SPEED_RPM;
      agitatorRPM = updateMotorToTarget(agitatorSim, agitatorRPM, agitatorTarget);

      // When indexer is active and shooter is at speed, periodically drop
      // shooter RPM to trigger shot detection in ShooterTelemetry
      boolean indexerActive = indexerRPM > 100;
      boolean shooterAtSpeed =
          shooterRPM > (ShooterConstants.TARGET_RPM - ShooterConstants.SPEED_TOLERANCE_RPM);
      double now = Timer.getFPGATimestamp();

      if (indexerActive && shooterAtSpeed && (now - lastShotSimTime) > SHOT_SIM_INTERVAL) {
        shooterRPM -= SHOT_RPM_DROP;
        shooterSim.getRelativeEncoderSim().setVelocity(shooterRPM);
        lastShotSimTime = now;
        shotFiredThisCycle = true;
        SafeLog.put("Sim/Debug/ShotSimulated", true);
      } else {
        shotFiredThisCycle = false;
        SafeLog.put("Sim/Debug/ShotSimulated", false);
      }

      // Refresh hold whenever we see a nonzero target >= current sticky.
      // This keeps hold alive during 50/0 oscillation (refreshed every other cycle).
      // Hold only expires after 10 consecutive zero-target cycles (real release).
      IntakePivot actuatorInst = IntakePivot.getInstance();
      if (actuatorInst == null) return;
      double rawActuatorTarget = actuatorInst.getTargetPosition();
      if (rawActuatorTarget != 0 && Math.abs(rawActuatorTarget) >= Math.abs(lastActuatorTarget)) {
        lastActuatorTarget = rawActuatorTarget;
        actuatorHoldCycles = TARGET_HOLD_COUNT;
      } else if (actuatorHoldCycles > 0) {
        actuatorHoldCycles--;
      } else {
        lastActuatorTarget = rawActuatorTarget;
      }
      intakeActuatorPos =
          updatePositionToTarget(intakeActuatorSim, intakeActuatorPos, lastActuatorTarget);

      Hanger hangerInst = Hanger.getInstance();
      if (hangerInst == null) return;
      double rawHangerTarget = hangerInst.getTargetPosition();
      if (rawHangerTarget != 0 && Math.abs(rawHangerTarget) >= Math.abs(lastHangerTarget)) {
        lastHangerTarget = rawHangerTarget;
        hangerHoldCycles = TARGET_HOLD_COUNT;
      } else if (hangerHoldCycles > 0) {
        hangerHoldCycles--;
      } else {
        lastHangerTarget = rawHangerTarget;
      }
      hangerPos = updatePositionToTarget(hangerSim, hangerPos, lastHangerTarget);

      SafeLog.put("Sim/Debug/ShooterOutput", shooterSim.getAppliedOutput());
      SafeLog.put("Sim/Debug/ShooterRPM", shooterRPM);
      SafeLog.put("Sim/Debug/ShooterTarget", shooterTarget);
      SafeLog.put("Sim/Debug/IndexerOutput", indexerSim.getAppliedOutput());
      SafeLog.put("Sim/Debug/IndexerRPM", indexerRPM);
      SafeLog.put("Sim/Debug/IntakeOutput", intakeSim.getAppliedOutput());
      SafeLog.put("Sim/Debug/IntakeRPM", intakeRPM);
      SafeLog.put("Sim/Debug/AgitatorOutput", agitatorSim.getAppliedOutput());
      SafeLog.put("Sim/Debug/AgitatorRPM", agitatorRPM);
      SafeLog.put("Sim/Debug/IntakeActuatorPos", intakeActuatorPos);
      SafeLog.put("Sim/Debug/IntakeActuatorTarget", lastActuatorTarget);
      SafeLog.put("Sim/Debug/HangerPos", hangerPos);
      SafeLog.put("Sim/Debug/HangerTarget", lastHangerTarget);
    } catch (RuntimeException e) {
      SafeLog.put("Sim/DeviceManager/UpdateError", e.getClass().getSimpleName());
    }
  }

  private double updateMotorToTarget(SparkSim sim, double currentRPM, double targetRPM) {
    double vbus = Math.max(1.0, RoboRioSim.getVInVoltage());

    double tau = (Math.abs(targetRPM) > Math.abs(currentRPM)) ? SPINUP_TAU : COASTDOWN_TAU;
    double alpha = 1.0 - Math.exp(-DT / tau);
    currentRPM += (targetRPM - currentRPM) * alpha;

    if (Math.abs(currentRPM) < 0.5) currentRPM = 0;

    // iterate() first (updates PID state), then setVelocity() to override
    // firmware averaging filter that would otherwise cap velocity at ~80%
    sim.iterate(currentRPM, vbus, DT);
    sim.getRelativeEncoderSim().setVelocity(currentRPM);

    return currentRPM;
  }

  private double updatePositionToTarget(SparkSim sim, double currentPos, double targetPos) {
    double alpha = 1.0 - Math.exp(-DT / POSITION_TAU);
    currentPos += (targetPos - currentPos) * alpha;
    sim.getRelativeEncoderSim().setPosition(currentPos);
    return currentPos;
  }

  public SparkSim getShooterSim() {
    return shooterSim;
  }

  public SparkSim getIndexerSim() {
    return indexerSim;
  }

  public SparkSim getIntakeSim() {
    return intakeSim;
  }

  public SparkSim getAgitatorSim() {
    return agitatorSim;
  }

  public SparkSim getIntakeActuatorSim() {
    return intakeActuatorSim;
  }

  public SparkSim getHangerSim() {
    return hangerSim;
  }

  public boolean isInitialized() {
    return initialized;
  }

  /** True if a shot was simulated this cycle (rising-edge for FuelPhysicsSim). */
  public boolean wasShotFiredThisCycle() {
    return shotFiredThisCycle;
  }

  /** Current simulated shooter RPM (for FuelPhysicsSim exit velocity). */
  public double getShooterRPM() {
    return shooterRPM;
  }
}
