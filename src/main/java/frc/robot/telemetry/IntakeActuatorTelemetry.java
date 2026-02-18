package frc.robot.telemetry;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants.DeviceHealthConstants;
import frc.robot.subsystems.IntakeActuator;

/** IntakeActuator telemetry: position and motor health. */
public class IntakeActuatorTelemetry implements SubsystemTelemetry {
  private IntakeActuator intakeActuator; // Not final - can re-acquire if null
  private boolean subsystemAvailable = false;

  private static final double POSITION_TOLERANCE = 0.05;

  private double positionRotations = 0;
  private double targetPosition = 0;
  private boolean atTarget = false;
  private double appliedOutput = 0;
  private double currentAmps = 0;
  private double temperatureCelsius = 0;

  // Device health,debounced to filter CAN bus transients
  private final Debouncer connectDebouncer =
      new Debouncer(DeviceHealthConstants.DISCONNECT_DEBOUNCE_SEC, Debouncer.DebounceType.kFalling);
  private boolean deviceConnected = false;
  private int deviceFaultsRaw = 0;

  public IntakeActuatorTelemetry() {
    this.intakeActuator = IntakeActuator.getInstance();
  }

  @Override
  public void update() {
    if (intakeActuator == null) {
      intakeActuator = IntakeActuator.getInstance();
    }

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
      deviceConnected = connectDebouncer.calculate(true);
      deviceFaultsRaw = intakeActuator.getStickyFaultsRaw();
    } catch (Throwable t) {
      deviceConnected = connectDebouncer.calculate(false);
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
  public double getTemperature() {
    return temperatureCelsius;
  }

  public boolean isDeviceConnected() {
    return deviceConnected;
  }

  public int getDeviceFaultsRaw() {
    return deviceFaultsRaw;
  }
}
