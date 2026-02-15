package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

// Detect stuck sensors. Create instance, call update(state) each cycle.
public class SensorHealthChecker {
  private static final double STARTUP_SUPPRESSION_SECONDS = 10.0;

  private final String name;
  private final Alert stuckAlert;
  private final double stuckTimeThreshold;

  private boolean lastState = false;
  private double lastStateChangeTime = 0;
  private boolean initialized = false;

  public SensorHealthChecker(String name, double stuckTimeSeconds) {
    this.name = name;
    this.stuckTimeThreshold = stuckTimeSeconds;
    this.stuckAlert = new Alert(name + " may be stuck", AlertType.kWarning);
  }

  // Returns true if healthy, false if stuck
  public boolean update(boolean currentState) {
    double now = Timer.getFPGATimestamp();

    if (!initialized) {
      lastState = currentState;
      lastStateChangeTime = now;
      initialized = true;
      return true;
    }

    if (currentState != lastState) {
      lastState = currentState;
      lastStateChangeTime = now;
    }

    double timeInState = now - lastStateChangeTime;
    boolean mayBeStuck = timeInState > stuckTimeThreshold;

    // Suppress alerts during startup to avoid false positives
    stuckAlert.set(mayBeStuck && now > STARTUP_SUPPRESSION_SECONDS);

    Logger.recordOutput("Sensors/" + name + "/State", currentState);
    Logger.recordOutput("Sensors/" + name + "/TimeInStateSeconds", timeInState);
    Logger.recordOutput("Sensors/" + name + "/MayBeStuck", mayBeStuck);

    return !mayBeStuck;
  }

  public void reset() {
    lastStateChangeTime = Timer.getFPGATimestamp();
    initialized = false;
  }
}
