package frc.robot.telemetry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants.DeviceHealthConstants;
import frc.robot.Constants.HangerConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.Hanger;

/** Hanger telemetry: position, deploy/climb state, motor health. */
public class HangerTelemetry implements SubsystemTelemetry {
    private Hanger hanger;  // Not final - can re-acquire if null
    private boolean subsystemAvailable = false;

    private double position = 0;
    private double targetPosition = 0;
    private double positionError = 0;
    private boolean atPosition = false;
    private double appliedOutput = 0;
    private double currentAmps = 0;
    private double temperatureCelsius = 0;
    private boolean isDeployed = false;
    private boolean isClimbing = false;

    // Device health — debounced to filter CAN bus transients
    private final Debouncer connectDebouncer = new Debouncer(
        DeviceHealthConstants.DISCONNECT_DEBOUNCE_SEC, Debouncer.DebounceType.kFalling);
    private boolean deviceConnected = false;
    private int deviceFaultsRaw = 0;

    private double climbProgress = 0;

    // Extra diagnostics
    private String controlMode = "UNKNOWN";
    private boolean softLimitActive = false;

    public HangerTelemetry() {
        this.hanger = Hanger.getInstance();
    }

    @Override
    public void update() {
        if (hanger == null) {
            hanger = Hanger.getInstance();
        }

        if (hanger == null) {
            subsystemAvailable = false;
            setDefaultValues();
            return;
        }

        subsystemAvailable = true;

        try {
            position = hanger.getPosition();
            targetPosition = hanger.getTargetPosition();
            positionError = targetPosition - position;
            atPosition = hanger.isAtPosition();
            appliedOutput = hanger.getAppliedOutput();
            currentAmps = hanger.getOutputCurrent();
            temperatureCelsius = hanger.getTemperature();

            isDeployed = Math.abs(position - MotorConstants.UP_HANGER_POS) < HangerConstants.POSITION_TOLERANCE;
            isClimbing = targetPosition == MotorConstants.DOWN_HANGER_POS && !atPosition;

            // Climb progress (0-1 normalized)
            double range = MotorConstants.UP_HANGER_POS - MotorConstants.DOWN_HANGER_POS;
            climbProgress = (range > 0) ? MathUtil.clamp((position - MotorConstants.DOWN_HANGER_POS) / range, 0, 1) : 0;

            // Device health
            deviceConnected = connectDebouncer.calculate(true);
            deviceFaultsRaw = hanger.getStickyFaultsRaw();

            // Control mode and soft limits
            controlMode = (targetPosition != 0) ? "POSITION" : "OPEN_LOOP";
            // Check if near position limits (use DOWN/UP constants as proxy)
            softLimitActive = (position <= MotorConstants.DOWN_HANGER_POS + 0.1)
                || (position >= MotorConstants.UP_HANGER_POS - 0.1);
        } catch (Throwable t) {
            deviceConnected = connectDebouncer.calculate(false);
            deviceFaultsRaw = -1;
            setDefaultValues();
        }
    }

    private void setDefaultValues() {
        position = 0;
        targetPosition = 0;
        positionError = 0;
        atPosition = false;
        appliedOutput = 0;
        currentAmps = 0;
        temperatureCelsius = 0;
        isDeployed = false;
        isClimbing = false;
    }

    @Override
    public void log() {
        SafeLog.put("Hanger/Available", subsystemAvailable);
        SafeLog.put("Hanger/Position", position);
        SafeLog.put("Hanger/TargetPosition", targetPosition);
        SafeLog.put("Hanger/PositionError", positionError);
        SafeLog.put("Hanger/AtPosition", atPosition);
        SafeLog.put("Hanger/AppliedOutput", appliedOutput);
        SafeLog.put("Hanger/CurrentAmps", currentAmps);
        SafeLog.put("Hanger/TemperatureCelsius", temperatureCelsius);
        SafeLog.put("Hanger/IsDeployed", isDeployed);
        SafeLog.put("Hanger/IsClimbing", isClimbing);

        // Device health
        SafeLog.put("Hanger/Device/Connected", deviceConnected);
        SafeLog.put("Hanger/Device/FaultsRaw", deviceFaultsRaw);

        SafeLog.put("Hanger/ClimbProgress", climbProgress);

        // Extra diagnostics
        SafeLog.put("Hanger/ControlMode", controlMode);
        SafeLog.put("Hanger/SoftLimitActive", softLimitActive);
    }

    @Override
    public String getName() {
        return "Hanger";
    }

    // Accessors for TelemetryManager
    public double getTemperature() { return temperatureCelsius; }
}
