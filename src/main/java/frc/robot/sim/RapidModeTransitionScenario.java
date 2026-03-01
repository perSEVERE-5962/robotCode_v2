package frc.robot.sim;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.telemetry.SafeLog;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Rapid mode transition stress test: exercises MatchTelemetry, CommandsTelemetry, and
 * mode-dependent logic under quick enable/disable cycling.
 *
 * <p>Tests: mode transition handling, per-phase statistics (MatchStatsTelemetry), command
 * scheduling stability, PostMatchSummary under non-standard conditions.
 *
 * <p>30s total. Simulates field disconnect/reconnect, rapid auto↔teleop switching, and the kind of
 * choppy FMS behavior teams see in practice.
 *
 * <p>Launch: ./gradlew simulateJava -DsimScenario=ModeTransition
 */
public class RapidModeTransitionScenario implements SimScenario {
  private double startTime;
  private boolean finished = false;
  private int lastPhase = -1;
  private int transitionCount = 0;

  @Override
  public String getName() {
    return "ModeTransition";
  }

  @Override
  public void init() {
    startTime = Timer.getFPGATimestamp();
    finished = false;
    lastPhase = -1;
    transitionCount = 0;
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
      SafeLog.put("Sim/ModeTransit/Phase", phase);
      SafeLog.put("Sim/ModeTransit/PhaseName", phaseName(phase));
      lastPhase = phase;
    }

    onPhaseUpdate(phase, t);

    SafeLog.put("Sim/ModeTransit/ElapsedSec", t);
    SafeLog.put("Sim/ModeTransit/Transitions", transitionCount);

    if (t >= 30.0) {
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
    if (t < 2) return 0; // Initial disabled
    if (t < 5) return 1; // Quick auto enable
    if (t < 6) return 2; // Disable (field glitch)
    if (t < 9) return 3; // Teleop enable
    if (t < 10) return 4; // Disable (field glitch)
    if (t < 13) return 5; // Back to teleop
    if (t < 14) return 6; // Disable
    if (t < 20) return 7; // Sustained teleop (normal operation)
    if (t < 21) return 8; // Disable
    if (t < 24) return 9; // Auto re-enable (weird FMS)
    if (t < 25) return 10; // Disable
    if (t < 28) return 11; // Final teleop
    return 12; // Final disable
  }

  private String phaseName(int phase) {
    switch (phase) {
      case 0:
        return "InitialDisabled";
      case 1:
        return "AutoEnable";
      case 2:
        return "FieldGlitch1";
      case 3:
        return "TeleopEnable";
      case 4:
        return "FieldGlitch2";
      case 5:
        return "TeleopResume";
      case 6:
        return "BriefDisable";
      case 7:
        return "SustainedTeleop";
      case 8:
        return "MidMatchDisable";
      case 9:
        return "AutoReEnable";
      case 10:
        return "FinalDisable1";
      case 11:
        return "FinalTeleop";
      case 12:
        return "MatchEnd";
      default:
        return "Unknown";
    }
  }

  private void onPhaseUpdate(int phase, double t) {
    switch (phase) {
      case 0: // Disabled
        setMode(false, false, 0);
        break;

      case 1: // Auto
        setMode(true, true, 15.0 - (t - 2.0));
        break;

      case 2: // Disabled (glitch)
        setMode(false, false, 0);
        break;

      case 3: // Teleop
        setMode(true, false, 135.0 - (t - 6.0));
        break;

      case 4: // Disabled (glitch)
        setMode(false, false, 0);
        break;

      case 5: // Teleop resume
        setMode(true, false, 130.0 - (t - 10.0));
        break;

      case 6: // Brief disable
        setMode(false, false, 0);
        break;

      case 7: // Sustained teleop,6s of normal operation
        setMode(true, false, 120.0 - (t - 14.0));
        break;

      case 8: // Mid-match disable
        setMode(false, false, 0);
        break;

      case 9: // Auto re-enable (unusual)
        setMode(true, true, 10.0 - (t - 21.0));
        break;

      case 10: // Final disable
        setMode(false, false, 0);
        break;

      case 11: // Final teleop
        setMode(true, false, 30.0 - (t - 25.0));
        break;

      case 12: // Match end
        setMode(false, false, 0);
        break;
    }
  }

  private void setMode(boolean enabled, boolean auto, double matchTime) {
    boolean wasEnabled = DriverStationSim.getEnabled();
    boolean wasAuto = DriverStationSim.getAutonomous();

    if (wasEnabled != enabled || wasAuto != auto) {
      transitionCount++;
    }

    DriverStationSim.setEnabled(enabled);
    DriverStationSim.setAutonomous(auto);
    if (matchTime > 0) {
      DriverStationSim.setMatchTime(matchTime);
    }
    DriverStationSim.notifyNewData();
  }
}
