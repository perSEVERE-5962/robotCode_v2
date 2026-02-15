package frc.robot.telemetry;

import edu.wpi.first.wpilibj.DriverStation;

/** Match state: time, mode, FMS, alliance. */
public class MatchTelemetry implements SubsystemTelemetry {

  // Current state
  private double matchTime = 0;
  private String mode = "DISABLED";
  private boolean enabled = false;
  private boolean fmsAttached = false;
  private int matchNumber = 0;
  private String eventName = "";
  private String alliance = "UNKNOWN";
  private int stationNumber = 0;

  public MatchTelemetry() {}

  @Override
  public void update() {
    matchTime = DriverStation.getMatchTime();
    enabled = DriverStation.isEnabled();
    fmsAttached = DriverStation.isFMSAttached();
    matchNumber = DriverStation.getMatchNumber();
    eventName = DriverStation.getEventName();
    stationNumber = DriverStation.getLocation().orElse(0);

    if (DriverStation.isDisabled()) {
      mode = "DISABLED";
    } else if (DriverStation.isAutonomous()) {
      mode = "AUTO";
    } else if (DriverStation.isTest()) {
      mode = "TEST";
    } else if (DriverStation.isTeleop()) {
      mode = "TELEOP";
    } else {
      mode = "UNKNOWN";
    }

    var allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isPresent()) {
      alliance = allianceOpt.get().name();
    } else {
      alliance = "UNKNOWN";
    }
  }

  @Override
  public void log() {
    SafeLog.put("Match/Time", matchTime);
    SafeLog.put("Match/Mode", mode);
    SafeLog.put("Match/Enabled", enabled);
    SafeLog.put("Match/FMSAttached", fmsAttached);
    SafeLog.put("Match/MatchNumber", matchNumber);
    SafeLog.put("Match/EventName", eventName);
    SafeLog.put("Match/Alliance", alliance);
    SafeLog.put("Match/StationNumber", stationNumber);
  }

  @Override
  public String getName() {
    return "Match";
  }
}
