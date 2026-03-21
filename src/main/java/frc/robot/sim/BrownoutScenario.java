package frc.robot.sim;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.telemetry.SafeLog;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Brownout scenario: runs motors under load then ramps voltage down to exercise SystemHealth
 * brownout detection and PredictiveAlerts BatteryAtRisk prediction, then recovers.
 *
 * <p>Two-stage voltage decline: Stage 1 (slow, 12.5V to 11.0V over 8s) triggers PredictiveAlerts
 * BatteryAtRisk prediction (needs 150 samples at 50Hz = 3s to fill regression window, then dropRate
 * > 0.001 V/s with predictedTimeToWarning < 30s). Stage 2 (fast, 11.0V to 7.0V over 3s) triggers
 * SystemHealth brownout detection and CRITICAL_ALERT LED state.
 *
 * <p>25s total, 7 phases.
 *
 * <p>Launch: ./gradlew simulateJava -DsimScenario=Brownout
 */
public class BrownoutScenario implements SimScenario {
  private double startTime;
  private boolean finished = false;
  private final SimInputPlayback input = new SimInputPlayback();
  private int lastPhase = -1;

  @Override
  public String getName() {
    return "Brownout";
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
      SafeLog.put("Sim/Brownout/Phase", phase);
      SafeLog.put("Sim/Brownout/PhaseName", phaseName(phase));
      lastPhase = phase;
      onPhaseEnter(phase);
    }

    onPhaseUpdate(phase, t);

    SafeLog.put("Sim/Brownout/ElapsedSec", t);
    SafeLog.put("Sim/Brownout/Progress", Math.min(100, (t / 25.0) * 100));

    if (t >= 25.0) {
      input.releaseAll();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      RoboRioSim.setVInVoltage(12.5);
      finished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  private int getPhase(double t) {
    if (t < 3) return 0; // Enable + drive + shooter
    if (t < 5) return 1; // Add indexer, all motors loaded
    if (t < 13) return 2; // Stage 1: slow ramp 12.5V -> 11.0V (triggers PredictiveAlerts)
    if (t < 16) return 3; // Stage 2: fast ramp 11.0V -> 7.0V (triggers brownout)
    if (t < 18) return 4; // Hold at 7V (CRITICAL_ALERT)
    if (t < 22) return 5; // Ramp 7V -> 12.5V recovery
    return 6; // Release + disable
  }

  private String phaseName(int phase) {
    switch (phase) {
      case 0:
        return "EnableAndLoad";
      case 1:
        return "FullMotorLoad";
      case 2:
        return "SlowVoltageDecline";
      case 3:
        return "FastBrownoutDrop";
      case 4:
        return "BrownoutHold";
      case 5:
        return "VoltageRecovery";
      case 6:
        return "Disable";
      default:
        return "Unknown";
    }
  }

  private void onPhaseEnter(int phase) {
    switch (phase) {
      case 0: // Enable teleop, drive + spin shooter
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setMatchTime(135.0);
        DriverStationSim.notifyNewData();
        input.driveForward(0.5);
        input.holdA(true);
        break;

      case 1: // Add indexer
        input.holdY(true);
        break;

      case 6: // Disable
        input.releaseAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        break;
    }
  }

  private void onPhaseUpdate(int phase, double t) {
    switch (phase) {
      case 2: // Stage 1: slow ramp 12.5V -> 11.0V over 8s (0.19 V/s)
        // PredictiveAlerts needs 150 samples (3s) to fill regression window,
        // then dropRate (0.19 V/s) >> 0.001 threshold fires BatteryAtRisk
        // when predictedTimeToWarning = (currentV - 11.5) / 0.19 < 30s
        double slowProgress = (t - 5.0) / 8.0;
        double slowVoltage = 12.5 - (1.5 * Math.min(1.0, slowProgress));
        RoboRioSim.setVInVoltage(slowVoltage);
        SafeLog.put("Sim/BrownoutVoltage", slowVoltage);
        break;

      case 3: // Stage 2: fast ramp 11.0V -> 7.0V over 3s
        double fastProgress = (t - 13.0) / 3.0;
        double fastVoltage = 11.0 - (4.0 * Math.min(1.0, fastProgress));
        RoboRioSim.setVInVoltage(fastVoltage);
        SafeLog.put("Sim/BrownoutVoltage", fastVoltage);
        break;

      case 4: // Hold at 7V (CRITICAL_ALERT territory)
        RoboRioSim.setVInVoltage(7.0);
        SafeLog.put("Sim/BrownoutVoltage", 7.0);
        break;

      case 5: // Ramp 7V -> 12.5V over 4s
        double recovProgress = (t - 18.0) / 4.0;
        double recovVoltage = 7.0 + (5.5 * Math.min(1.0, recovProgress));
        RoboRioSim.setVInVoltage(recovVoltage);
        SafeLog.put("Sim/BrownoutVoltage", recovVoltage);
        break;
    }
  }
}
