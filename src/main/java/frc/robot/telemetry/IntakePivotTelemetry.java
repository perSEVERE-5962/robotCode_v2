package frc.robot.telemetry;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DeviceHealthConstants;
import frc.robot.subsystems.IntakePivot;

/** IntakePivot telemetry: position and motor health. */
public class IntakePivotTelemetry implements SubsystemTelemetry {
  private IntakePivot intakePivot; // grabbed again in update() if not ready yet
  private boolean subsystemAvailable = false;

  private static final double POSITION_TOLERANCE = 0.05;

  private double positionRotations = 0;
  private double targetPosition = 0;
  private boolean atTarget = false;
  private double appliedOutput = 0;
  private double currentAmps = 0;
  private double temperatureCelsius = 0;

  private final Debouncer connectDebouncer =
      new Debouncer(DeviceHealthConstants.DISCONNECT_DEBOUNCE_SEC, Debouncer.DebounceType.kFalling);
  private boolean deviceConnected = false;
  private int deviceFaultsRaw = 0;

  private String activeCommandName = "none";

  public IntakePivotTelemetry() {
    this.intakePivot = IntakePivot.getInstance();
  }

  @Override
  public void update() {
    if (intakePivot == null) {
      intakePivot = IntakePivot.getInstance();
    }

    if (intakePivot == null) {
      subsystemAvailable = false;
      setDefaultValues();
      return;
    }

    subsystemAvailable = true;

    try {
      positionRotations = intakePivot.getPosition();
      targetPosition = intakePivot.getTargetPosition();
      atTarget = intakePivot.isAtTarget();
      appliedOutput = intakePivot.getAppliedOutput();
      currentAmps = intakePivot.getOutputCurrent();
      temperatureCelsius = intakePivot.getTemperature();

      deviceConnected = connectDebouncer.calculate(true);
      deviceFaultsRaw = intakePivot.getStickyFaultsRaw();
    } catch (Throwable t) {
      deviceConnected = connectDebouncer.calculate(false);
      deviceFaultsRaw = -1;
      setDefaultValues();
    }

    try {
      Command currentCmd = intakePivot.getCurrentCommand();
      activeCommandName = (currentCmd != null) ? currentCmd.getName() : "none";
    } catch (Throwable t) {
      activeCommandName = "unknown";
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
    SafeLog.put("IntakePivot/Available", subsystemAvailable);
    SafeLog.put("IntakePivot/PositionRotations", positionRotations);
    SafeLog.put("IntakePivot/TargetPosition", targetPosition);
    SafeLog.put("IntakePivot/AtTarget", atTarget);
    SafeLog.put("IntakePivot/AppliedOutput", appliedOutput);
    SafeLog.put("IntakePivot/CurrentAmps", currentAmps);
    SafeLog.put("IntakePivot/TemperatureCelsius", temperatureCelsius);

    SafeLog.put("IntakePivot/Device/Connected", deviceConnected);
    SafeLog.put("IntakePivot/Device/FaultsRaw", deviceFaultsRaw);
    SafeLog.put("IntakePivot/ActiveCommand", activeCommandName);
  }

  @Override
  public String getName() {
    return "IntakePivot";
  }

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
