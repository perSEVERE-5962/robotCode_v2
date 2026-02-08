package frc.robot.telemetry;

/** Common interface for telemetry classes. */
public interface SubsystemTelemetry {
    void update();

    void log();

    String getName();
}
