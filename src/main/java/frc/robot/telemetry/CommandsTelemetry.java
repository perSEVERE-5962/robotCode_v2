package frc.robot.telemetry;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.HashMap;
import java.util.IdentityHashMap;
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
}
