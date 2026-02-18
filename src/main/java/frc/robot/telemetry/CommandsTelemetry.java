package frc.robot.telemetry;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

/** Command scheduler telemetry: active commands, execution counts, durations. */
public class CommandsTelemetry implements SubsystemTelemetry {
  private final Map<String, Integer> activeCommandCounts = new HashMap<>();
  private String activeList = "none";
  private int activeCount = 0;

  // Performance tracking
  private final Map<Command, Double> commandStartTimes = new IdentityHashMap<>();
  private final Map<String, Integer> commandExecutionCounts = new HashMap<>();
  private static final int MAX_EXECUTION_COUNT_ENTRIES = 100;
  private int totalExecutions = 0;
  private String lastCommandName = "none";
  private double lastCommandDurationMs = 0;

  // Command stability metrics
  private int interruptedCount = 0;
  private int finishedCount = 0;

  // Ghost command detection: commands running >GHOST_THRESHOLD_SEC without finishing.
  // Catches orphaned commands scheduled from end() that run forever (e.g. MoveShooter(0)).
  private static final double GHOST_THRESHOLD_SEC = 5.0;
  private static final int MAX_GHOST_ENTRIES = 20;
  private final Map<Command, String> commandNames = new IdentityHashMap<>();
  private String ghostCommandList = "none";
  private int ghostCommandCount = 0;
  private boolean ghostCommandAlert = false;

  public CommandsTelemetry() {
    setupCallbacks();
  }

  private String sanitizeName(Command command) {
    if (command == null) return "NullCommand";
    String name = command.getName();
    return (name != null && !name.isEmpty()) ? name : command.getClass().getSimpleName();
  }

  private void setupCallbacks() {
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> {
              try {
                String name = sanitizeName(command);
                activeCommandCounts.merge(name, 1, Integer::sum);

                // Track start time by command identity (not name)
                commandStartTimes.put(command, Timer.getFPGATimestamp());

                // Track name for ghost detection
                if (commandNames.size() < MAX_GHOST_ENTRIES) {
                  commandNames.put(command, name);
                }

                // Bound execution counts map
                if (commandExecutionCounts.size() < MAX_EXECUTION_COUNT_ENTRIES) {
                  commandExecutionCounts.merge(name, 1, Integer::sum);
                }
                totalExecutions++;
              } catch (Throwable t) {
              }
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            command -> {
              try {
                String name = sanitizeName(command);
                activeCommandCounts.compute(name, (k, v) -> (v == null || v <= 1) ? null : v - 1);

                // Track duration by command identity
                Double startTime = commandStartTimes.remove(command);
                if (startTime != null) {
                  lastCommandDurationMs = (Timer.getFPGATimestamp() - startTime) * 1000;
                  lastCommandName = name;
                }
                commandNames.remove(command);

                finishedCount++;
              } catch (Throwable t) {
              }
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> {
              try {
                String name = sanitizeName(command);
                activeCommandCounts.compute(name, (k, v) -> (v == null || v <= 1) ? null : v - 1);

                // Track duration by command identity
                Double startTime = commandStartTimes.remove(command);
                if (startTime != null) {
                  lastCommandDurationMs = (Timer.getFPGATimestamp() - startTime) * 1000;
                  lastCommandName = name + " (interrupted)";
                }
                commandNames.remove(command);

                // Track interruptions for stability analysis
                interruptedCount++;
              } catch (Throwable t) {
              }
            });
  }

  @Override
  public void update() {
    if (activeCommandCounts.isEmpty()) {
      activeList = "none";
    } else {
      StringBuilder sb = new StringBuilder();
      for (String name : activeCommandCounts.keySet()) {
        if (activeCommandCounts.getOrDefault(name, 0) > 0) {
          if (sb.length() > 0) sb.append(", ");
          sb.append(name);
        }
      }
      activeList = sb.length() > 0 ? sb.toString() : "none";
    }
    activeCount = activeCommandCounts.size();

    // Ghost command detection: find commands running longer than threshold
    double now = Timer.getFPGATimestamp();
    List<String> ghosts = new ArrayList<>();
    Iterator<Map.Entry<Command, Double>> it = commandStartTimes.entrySet().iterator();
    while (it.hasNext()) {
      Map.Entry<Command, Double> entry = it.next();
      double runSec = now - entry.getValue();
      if (runSec >= GHOST_THRESHOLD_SEC) {
        String name = commandNames.getOrDefault(entry.getKey(), "unknown");
        ghosts.add(name + " (" + String.format("%.0f", runSec) + "s)");
      }
    }
    ghostCommandCount = ghosts.size();
    ghostCommandAlert = ghostCommandCount > 0;
    ghostCommandList = ghosts.isEmpty() ? "none" : String.join(", ", ghosts);
  }

  @Override
  public void log() {
    SafeLog.put("Commands/ActiveList", activeList);
    SafeLog.put("Commands/ActiveCount", activeCount);
    SafeLog.put("Commands/TotalExecutions", totalExecutions);
    SafeLog.put("Commands/LastCommandName", lastCommandName);
    SafeLog.put("Commands/LastCommandDurationMs", lastCommandDurationMs);

    // Command stability metrics
    SafeLog.put("Commands/InterruptedCount", interruptedCount);
    SafeLog.put("Commands/FinishedCount", finishedCount);

    // Ghost command detection
    SafeLog.put("Commands/GhostList", ghostCommandList);
    SafeLog.put("Commands/GhostCount", ghostCommandCount);
    SafeLog.put("Commands/GhostAlert", ghostCommandAlert);
  }

  @Override
  public String getName() {
    return "Commands";
  }

  // Accessors
  public int getTotalExecutions() {
    return totalExecutions;
  }

  public double getLastCommandDurationMs() {
    return lastCommandDurationMs;
  }

  public int getGhostCommandCount() {
    return ghostCommandCount;
  }

  public boolean isGhostCommandAlert() {
    return ghostCommandAlert;
  }
}
