package frc.robot.sim;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.telemetry.SafeLog;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Fault injection scenario: tests brownout prediction, heavy load behavior, and recovery from
 * degraded conditions.
 *
 * <p>60s total, 6 phases. Walks voltage through all 4 brownout risk levels (0=none, 1=watch,
 * 2=warning, 3=imminent) while motors are running, then recovers.
 *
 * <p>Exercises: SystemHealth (brownout risk, voltage slope, CAN status), Shooter/Indexer/Intake
 * (behavior under low voltage), AlertManager, PredictiveAlerts.
 *
 * <p>Launch: ./gradlew simulateJava -DsimScenario=FaultInjection
 */
public class FaultInjectionScenario implements SimScenario {
  private double startTime;
  private boolean finished = false;
  private final SimInputPlayback input = new SimInputPlayback();
  private int lastPhase = -1;

  @Override
  public String getName() {
    return "FaultInjection";
  }

  @Override
  public void init() {
    startTime = Timer.getFPGATimestamp();
    finished = false;
    lastPhase = -1;
    input.releaseAll();
    SimulatedBattery.disableBatterySim(); // prevent MapleSim from overwriting our voltage ramp
    RoboRioSim.setVInVoltage(12.5);
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
  }

  @Override
  public void execute(double timestampSec) {
    double t = timestampSec - startTime;
    int phase = getPhase(t);

    if (phase != lastPhase) {
      SafeLog.put("Sim/FaultInject/Phase", phase);
      SafeLog.put("Sim/FaultInject/PhaseName", phaseName(phase));
      lastPhase = phase;
      onPhaseEnter(phase);
    }

    onPhaseUpdate(phase, t);

    SafeLog.put("Sim/FaultInject/ElapsedSec", t);
    SafeLog.put("Sim/FaultInject/Progress", Math.min(100, (t / 60.0) * 100));

    if (t >= 60.0) {
      input.releaseAll();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      finished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  private int getPhase(double t) {
    if (t < 5) return 0; // Healthy baseline
    if (t < 15) return 1; // Gradual voltage drop to 10V (risk level 0→1)
    if (t < 25) return 2; // Deep voltage drop to 8V (risk level 2→3)
    if (t < 30) return 3; // Hold at critical (7.5V),brownout imminent
    if (t < 40) return 4; // Recovery ramp back to 12.5V
    if (t < 55) return 5; // Heavy load: all motors + moderate voltage (10.5V)
    return 6; // Final recovery + disable
  }

  private String phaseName(int phase) {
    switch (phase) {
      case 0:
        return "HealthyBaseline";
      case 1:
        return "GradualDrop";
      case 2:
        return "DeepDrop";
      case 3:
        return "CriticalHold";
      case 4:
        return "Recovery";
      case 5:
        return "HeavyLoad";
      case 6:
        return "FinalRecovery";
      default:
        return "Unknown";
    }
  }

  private void onPhaseEnter(int phase) {
    switch (phase) {
      case 0: // Healthy baseline,enable teleop, start shooter
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setMatchTime(135.0);
        DriverStationSim.notifyNewData();
        input.holdA(true); // spin shooter for baseline
        break;

      case 1: // Gradual drop,keep shooting
        break;

      case 2: // Deep drop,motors struggling
        break;

      case 3: // Critical hold,worst case
        break;

      case 4: // Recovery,release motors, ramp voltage back
        input.releaseAll();
        break;

      case 5: // Heavy load,all motors at reduced voltage
        input.holdA(true); // shooter
        input.holdY(true); // indexer
        input.driveForward(0.8); // drive hard
        break;

      case 6: // Final recovery
        input.releaseAll();
        break;
    }
  }

  private void onPhaseUpdate(int phase, double t) {
    switch (phase) {
      case 0: // Healthy: 12.5V steady
        RoboRioSim.setVInVoltage(12.5);
        break;

      case 1: // Gradual: 12.5V → 10V over 10s
        // Slope: -0.25 V/s,should trigger risk level 0, maybe 1
        double dropT = t - 5.0;
        double dropProgress = Math.min(1.0, dropT / 10.0);
        RoboRioSim.setVInVoltage(12.5 - (2.5 * dropProgress));
        DriverStationSim.setMatchTime(135.0 - t);
        DriverStationSim.notifyNewData();
        break;

      case 2: // Deep: 10V → 8V over 10s
        // Slope: -0.2 V/s,risk level 1→2→3 as voltage crosses thresholds
        double deepT = t - 15.0;
        double deepProgress = Math.min(1.0, deepT / 10.0);
        RoboRioSim.setVInVoltage(10.0 - (2.0 * deepProgress));
        DriverStationSim.setMatchTime(135.0 - t);
        DriverStationSim.notifyNewData();
        break;

      case 3: // Critical: hold at 7.5V
        // With motors running and slope < -2 V/s initially, risk level 3
        RoboRioSim.setVInVoltage(7.5);
        DriverStationSim.setMatchTime(135.0 - t);
        DriverStationSim.notifyNewData();
        break;

      case 4: // Recovery: 7.5V → 12.5V over 10s
        double recovT = t - 30.0;
        double recovProgress = Math.min(1.0, recovT / 10.0);
        RoboRioSim.setVInVoltage(7.5 + (5.0 * recovProgress));
        DriverStationSim.setMatchTime(135.0 - t);
        DriverStationSim.notifyNewData();
        break;

      case 5: // Heavy load: moderate voltage (10.5V) with all motors
        // All 3 motors running draws high current at reduced voltage
        RoboRioSim.setVInVoltage(10.5);
        input.rotate(0.3); // add rotation too
        DriverStationSim.setMatchTime(135.0 - t);
        DriverStationSim.notifyNewData();
        break;

      case 6: // Final: restore healthy voltage
        RoboRioSim.setVInVoltage(12.5);
        break;
    }
  }
}
