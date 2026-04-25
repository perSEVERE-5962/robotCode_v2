package frc.robot.sim;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.util.SafeLog;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Rapid-fire scenario: pre-spins shooter then fires via SpeedUpThenIndex in fast 1.5s cycles.
 * Exercises Shooter, Indexer signals.
 *
 * <p>13s total, 3 phases. Previously broken,never enabled the robot.
 *
 * <p>Launch: ./gradlew simulateJava -DsimScenario=RapidFire
 */
class RapidFireScenario implements SimScenario {
  private double startTime;
  private boolean finished = false;
  private final SimInputPlayback input = new SimInputPlayback();
  private int lastPhase = -1;
  private int shotCount = 0;
  private boolean xPressed = false;

  private static final double CYCLE_PERIOD = 1.5;
  private static final double SPIN_TIME = 1.0;
  private static final double FIRE_TIME = 0.3;

  @Override
  public String getName() {
    return "RapidFire";
  }

  @Override
  public void init() {
    startTime = Timer.getFPGATimestamp();
    finished = false;
    lastPhase = -1;
    shotCount = 0;
    xPressed = false;
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
      SafeLog.put("Sim/RapidFire/Phase", phase);
      SafeLog.put("Sim/RapidFire/PhaseName", phaseName(phase));
      lastPhase = phase;
      onPhaseEnter(phase);
    }

    onPhaseUpdate(phase, t);

    SafeLog.put("Sim/RapidFire/ElapsedSec", t);
    SafeLog.put("Sim/RapidFireShots", shotCount);
    SafeLog.put("Sim/RapidFireRate", t > 0 ? shotCount / t : 0);
    SafeLog.put("Sim/RapidFireProgress", Math.min(100, (t / 13.0) * 100));

    if (t >= 13.0) {
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
    if (t < 2) return 0; // Enable + pre-spin
    if (t < 12) return 1; // Rapid fire cycles
    return 2; // Release + disable
  }

  private String phaseName(int phase) {
    switch (phase) {
      case 0:
        return "PreSpin";
      case 1:
        return "RapidFire";
      case 2:
        return "Disable";
      default:
        return "Unknown";
    }
  }

  private void onPhaseEnter(int phase) {
    switch (phase) {
      case 0: // Enable teleop + pre-spin shooter
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setMatchTime(135.0);
        DriverStationSim.notifyNewData();
        input.holdA(true);
        break;

      case 2: // Disable
        input.releaseAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        break;
    }
  }

  private void onPhaseUpdate(int phase, double t) {
    if (phase != 1) return;

    // Rapid fire: 1.5s cycles (1.0s spin, 0.3s fire, 0.2s reset)
    double cyclePos = (t - 2.0) % CYCLE_PERIOD;

    if (cyclePos < SPIN_TIME) {
      // Spin-up: hold A, release X
      input.holdA(true);
      if (xPressed) {
        input.releaseX();
        xPressed = false;
      }
    } else if (cyclePos < SPIN_TIME + FIRE_TIME) {
      // Fire: tap X (SpeedUpThenIndex)
      if (!xPressed) {
        input.tapX();
        xPressed = true;
        shotCount++;
      }
    } else {
      // Reset: release X for next cycle
      if (xPressed) {
        input.releaseX();
        xPressed = false;
      }
    }
  }
}
