package frc.robot.telemetry;

/** Common interface for telemetry classes. */
interface SubsystemTelemetry {
  void update();

  void log();

  String getName();
}
