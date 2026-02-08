package frc.robot.telemetry;

import frc.robot.subsystems.IntakeActuator;

/** IntakeActuator telemetry: position and motor health. */
public class IntakeActuatorTelemetry implements SubsystemTelemetry {
    private IntakeActuator intakeActuator;  // Not final - can re-acquire if null
    private boolean subsystemAvailable = false;

    private static final double POSITION_TOLERANCE = 0.05;

    // Current state
    private double positionRotations = 0;
    private double targetPosition = 0;
    private boolean atTarget = false;
    private double appliedOutput = 0;
    private double currentAmps = 0;
    private double temperatureCelsius = 0;

    // Device health
    private boolean deviceConnected = false;
    private int deviceFaultsRaw = 0;

    public IntakeActuatorTelemetry() {
        this.intakeActuator = IntakeActuator.getInstance();
    }

    @Override
    public void update() {
        // Re-acquire subsystem if null
        if (intakeActuator == null) {
            intakeActuator = IntakeActuator.getInstance();
        }

        // Still null? Log safe defaults and return
        if (intakeActuator == null) {
            subsystemAvailable = false;
            setDefaultValues();
            return;
        }

        subsystemAvailable = true;

        try {
            positionRotations = intakeActuator.getPosition();
            targetPosition = intakeActuator.getTargetPosition();
            atTarget = intakeActuator.isAtTarget();
            appliedOutput = intakeActuator.getAppliedOutput();
            currentAmps = intakeActuator.getOutputCurrent();
            temperatureCelsius = intakeActuator.getTemperature();

            // Device health
            deviceConnected = true;
            deviceFaultsRaw = intakeActuator.getStickyFaultsRaw();
        } catch (Throwable t) {
            deviceConnected = false;
            deviceFaultsRaw = -1;
            setDefaultValues();
        }
    }

    private void setDefaultValues() {
        positionRotations = 0;
        targetPosition = 0;
        atTarget = false;
        appliedOutput = 0;
        currentAmps = 0;
        temperatureCelsius = 0;
    }

    @Override
    public void log() {
        SafeLog.put("IntakeActuator/Available", subsystemAvailable);
        SafeLog.put("IntakeActuator/PositionRotations", positionRotations);
        SafeLog.put("IntakeActuator/TargetPosition", targetPosition);
        SafeLog.put("IntakeActuator/AtTarget", atTarget);
        SafeLog.put("IntakeActuator/AppliedOutput", appliedOutput);
        SafeLog.put("IntakeActuator/CurrentAmps", currentAmps);
        SafeLog.put("IntakeActuator/TemperatureCelsius", temperatureCelsius);

        // Device health
        SafeLog.put("IntakeActuator/Device/Connected", deviceConnected);
        SafeLog.put("IntakeActuator/Device/FaultsRaw", deviceFaultsRaw);
    }

    @Override
    public String getName() {
        return "IntakeActuator";
    }

    // Accessors for TelemetryManager
    public double getTemperature() { return temperatureCelsius; }
}
