package frc.robot.sim;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.SafeLog;
import java.util.HashMap;
import java.util.Map;

public class SimScenarioRunner {
  private final Map<String, SimScenario> scenarios = new HashMap<>();
  private SimScenario activeScenario = null;
  private String lastRequestedScenario = "";
  private final boolean cliLaunched;
  private int exitDelayCycles = -1; // countdown after CLI scenario finishes

  public SimScenarioRunner() {
    register(new CompetitionMatchScenario());
    register(new SubsystemSuiteScenario());
    register(new RapidFireScenario());
    register(new BrownoutScenario());
    register(new FaultInjectionScenario());
    register(new RapidModeTransitionScenario());
    register(new FullVideoShowcaseScenario());
    register(new AMDAShowcaseScenario());
    register(new SignalCoverageScenario());
    register(new HubShiftPracticeScenario());

    // CLI support: -DsimScenario=SignalCoverage
    String cliScenario = System.getProperty("simScenario", "");
    cliLaunched = !cliScenario.isEmpty();
    if (cliLaunched) {
      SmartDashboard.putString("Sim/RunScenario", cliScenario);
    } else {
      SmartDashboard.putString("Sim/RunScenario", "");
    }
  }

  private void register(SimScenario scenario) {
    scenarios.put(scenario.getName(), scenario);
  }

  /** Call from Robot.simulationPeriodic() */
  public void update() {
    try {
      String requested = SmartDashboard.getString("Sim/RunScenario", "");

      if (!requested.equals(lastRequestedScenario)) {
        lastRequestedScenario = requested;
        activeScenario = scenarios.get(requested);
        if (activeScenario != null) {
          activeScenario.init();
        }
      }

      // Auto-exit: after CLI scenario completes, wait 5s for log flush then exit
      if (cliLaunched && exitDelayCycles >= 0) {
        exitDelayCycles--;
        if (exitDelayCycles == 0) {
          System.exit(0);
        }
        return;
      }

      if (activeScenario != null) {
        if (activeScenario.isFinished()) {
          SafeLog.put("Sim/ScenarioActive", false);
          SafeLog.put("Sim/ScenarioName", "COMPLETED: " + activeScenario.getName());
          activeScenario = null;
          if (cliLaunched) {
            exitDelayCycles = 250; // ~5s at 50Hz for log flush
          }
        } else {
          activeScenario.execute(Timer.getFPGATimestamp());
          SafeLog.put("Sim/ScenarioActive", true);
          SafeLog.put("Sim/ScenarioName", activeScenario.getName());
        }
      } else {
        SafeLog.put("Sim/ScenarioActive", false);
        SafeLog.put("Sim/ScenarioName", "");
      }
    } catch (RuntimeException e) {
      SafeLog.put("Sim/ScenarioRunner/Error", e.getClass().getSimpleName());
    }
  }
}
