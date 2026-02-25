package frc.robot.sim;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.telemetry.SafeLog;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Lean gap-filler scenario covering signals that CompetitionMatch, SubsystemSuite, Brownout, and
 * AMDAShowcase do not exercise.
 *
 * <p>Focus areas (no overlap with other scenarios):
 *
 * <ul>
 *   <li>Systematic voltage sweep through all 4 thresholds (WARNING 11.5V, predicted shutdown
 *       10.5V, CRITICAL 10.0V, deep brownout 7.0V) with recovery
 *   <li>All mode transitions in one run: disabled -> auto -> disabled -> teleop -> disabled -> test
 *       -> disabled
 *   <li>Motor stall/recovery cycles: all motors at speed, then sudden target zero, then resume
 *       (exercises stall detection in ShooterTelemetry, IndexerTelemetry, IntakeTelemetry, etc.)
 * </ul>
 *
 * <p>30s total, 8 phases. Launch: ./gradlew simulateJava -DsimScenario=SignalCoverage
 */
public class SignalCoverageScenario implements SimScenario {
  private double startTime;
  private boolean finished = false;
  private final SimInputPlayback input = new SimInputPlayback();
  private int lastPhase = -1;

  private static final double MODE_CYCLE_END = 8.0;
  private static final double MOTOR_SPINUP_END = 13.0;
  private static final double MOTOR_STALL_END = 16.0;
  private static final double MOTOR_RECOVERY_END = 19.0;
  private static final double VOLTAGE_SWEEP_END = 26.0;
  private static final double VOLTAGE_RECOVER_END = 29.0;
  private static final double SCENARIO_END = 30.0;

  @Override
  public String getName() {
    return "SignalCoverage";
  }

  @Override
  public void init() {
    startTime = Timer.getFPGATimestamp();
    finished = false;
    lastPhase = -1;
    input.releaseAll();
    SimulatedBattery.disableBatterySim();
    RoboRioSim.setVInVoltage(12.8);
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
  }

  @Override
  public void execute(double timestampSec) {
    double t = timestampSec - startTime;
    int phase = getPhase(t);

    if (phase != lastPhase) {
      SafeLog.put("Sim/SigCov/Phase", phase);
      SafeLog.put("Sim/SigCov/PhaseName", phaseName(phase));
      lastPhase = phase;
      onPhaseEnter(phase);
    }

    onPhaseUpdate(phase, t);

    SafeLog.put("Sim/SigCov/ElapsedSec", t);
    SafeLog.put("Sim/SigCov/Progress", Math.min(100, (t / SCENARIO_END) * 100));

    if (t >= SCENARIO_END) {
      input.releaseAll();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      RoboRioSim.setVInVoltage(12.8);
      finished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  private int getPhase(double t) {
    if (t < MODE_CYCLE_END) return 0;
    if (t < MOTOR_SPINUP_END) return 1;
    if (t < MOTOR_STALL_END) return 2;
    if (t < MOTOR_RECOVERY_END) return 3;
    if (t < VOLTAGE_SWEEP_END) return 4;
    if (t < VOLTAGE_RECOVER_END) return 5;
    return 6;
  }

  private String phaseName(int phase) {
    switch (phase) {
      case 0:
        return "ModeCycle";
      case 1:
        return "MotorSpinUp";
      case 2:
        return "MotorStall";
      case 3:
        return "MotorRecovery";
      case 4:
        return "VoltageSweep";
      case 5:
        return "VoltageRecover";
      case 6:
        return "PostDisable";
      default:
        return "Unknown";
    }
  }

  private void onPhaseEnter(int phase) {
    switch (phase) {
      case 0: // Mode cycling handled in onPhaseUpdate
        DriverStationSim.setEnabled(false);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        break;

      case 1: // Spin up all motors in teleop
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setMatchTime(120.0);
        DriverStationSim.notifyNewData();
        // Driver: drive + shooter(A) + indexer+agitator(Y) + intake(B)
        input.driveForward(0.6);
        input.holdA(true);
        input.holdY(true);
        input.holdB(true);
        // Copilot: intake(A) + agitate(B)
        input.copilotHoldA(true);
        input.copilotHoldB(true);
        SafeLog.put("Sim/SigCov/MotorsLoaded", true);
        break;

      case 2: // Stall: release all motor commands suddenly
        input.holdA(false);
        input.holdY(false);
        input.holdB(false);
        input.copilotHoldA(false);
        input.copilotHoldB(false);
        input.driveForward(0);
        SafeLog.put("Sim/SigCov/MotorsLoaded", false);
        SafeLog.put("Sim/SigCov/StallPhase", true);
        break;

      case 3: // Recovery: spin motors back up
        input.driveForward(0.5);
        input.holdA(true);
        input.holdY(true);
        input.holdB(true);
        SafeLog.put("Sim/SigCov/StallPhase", false);
        SafeLog.put("Sim/SigCov/MotorsLoaded", true);
        break;

      case 4: // Voltage sweep (motors still running under load)
        break;

      case 5: // Voltage recovery
        break;

      case 6: // Post-disable
        input.releaseAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        break;
    }
  }

  private void onPhaseUpdate(int phase, double t) {
    switch (phase) {
      case 0: // Mode cycle: disabled -> auto -> disabled -> teleop -> disabled -> test -> disabled
        modeCycle(t);
        break;

      case 4: // Voltage sweep: 12.8V -> 11.5 -> 10.5 -> 10.0 -> 7.0V over 7s
        double sweepT = t - MOTOR_RECOVERY_END;
        double sweepFrac = sweepT / (VOLTAGE_SWEEP_END - MOTOR_RECOVERY_END);
        double voltage;
        if (sweepFrac < 0.2) {
          // 12.8 -> 11.5 (WARNING threshold)
          voltage = 12.8 - (1.3 * (sweepFrac / 0.2));
        } else if (sweepFrac < 0.4) {
          // 11.5 -> 10.5 (predicted shutdown)
          voltage = 11.5 - (1.0 * ((sweepFrac - 0.2) / 0.2));
        } else if (sweepFrac < 0.6) {
          // 10.5 -> 10.0 (CRITICAL threshold)
          voltage = 10.5 - (0.5 * ((sweepFrac - 0.4) / 0.2));
        } else if (sweepFrac < 0.8) {
          // 10.0 -> 7.0 (deep brownout)
          voltage = 10.0 - (3.0 * ((sweepFrac - 0.6) / 0.2));
        } else {
          // Hold at 7.0
          voltage = 7.0;
        }
        RoboRioSim.setVInVoltage(voltage);
        SafeLog.put("Sim/SigCov/Voltage", voltage);
        break;

      case 5: // Recovery: 7.0 -> 12.8V over 3s
        double recT = t - VOLTAGE_SWEEP_END;
        double recFrac = Math.min(1.0, recT / (VOLTAGE_RECOVER_END - VOLTAGE_SWEEP_END));
        double recVoltage = 7.0 + (5.8 * recFrac);
        RoboRioSim.setVInVoltage(recVoltage);
        SafeLog.put("Sim/SigCov/Voltage", recVoltage);
        break;
    }
  }

  /**
   * Rapid mode cycling: disabled -> auto(1.5s) -> disabled(0.5s) -> teleop(2s) -> disabled(0.5s)
   * -> test(2s) -> disabled(1s). Exercises EventMarker.modeChange(),
   * PostMatchSummary.startTracking(), and all mode-dependent telemetry paths.
   */
  private void modeCycle(double t) {
    if (t < 0.5) {
      // Initial disabled
      DriverStationSim.setEnabled(false);
      DriverStationSim.setAutonomous(false);
    } else if (t < 2.0) {
      // Auto
      DriverStationSim.setAutonomous(true);
      DriverStationSim.setEnabled(true);
      DriverStationSim.setMatchTime(15.0);
      input.driveForward(0.3);
    } else if (t < 2.5) {
      // Disabled gap
      input.releaseAll();
      DriverStationSim.setEnabled(false);
      DriverStationSim.setAutonomous(false);
    } else if (t < 4.5) {
      // Teleop
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setEnabled(true);
      DriverStationSim.setMatchTime(135.0);
      input.driveForward(0.4);
    } else if (t < 5.0) {
      // Disabled gap
      input.releaseAll();
      DriverStationSim.setEnabled(false);
      DriverStationSim.setAutonomous(false);
    } else if (t < 7.0) {
      // Test mode
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setTest(true);
      DriverStationSim.setEnabled(true);
      input.driveForward(0.2);
    } else {
      // Final disabled
      input.releaseAll();
      DriverStationSim.setTest(false);
      DriverStationSim.setEnabled(false);
    }
    DriverStationSim.notifyNewData();
  }
}
