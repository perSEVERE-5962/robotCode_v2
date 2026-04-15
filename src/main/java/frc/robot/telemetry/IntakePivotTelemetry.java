package frc.robot.telemetry;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DeviceHealthConstants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.util.DeviceFaultDecoder;
import frc.robot.util.DeviceFaultDecoder.DecodedFaults;
import frc.robot.util.DeviceFaultDecoder.DeviceType;

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
  private double busVoltage = 0;
  private double voltageDropVolts = 0;
  private double temperatureCelsius = 0;

  private final Debouncer connectDebouncer =
      new Debouncer(DeviceHealthConstants.DISCONNECT_DEBOUNCE_SEC, Debouncer.DebounceType.kFalling);
  private boolean deviceConnected = false;
  private int deviceFaultsRaw = 0;
  private int lastRawFaults = 0;
  private int faultTransitionCount = 0;
  private DecodedFaults decodedFaults = DeviceFaultDecoder.empty();

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
      busVoltage = intakePivot.getBusVoltage();
      voltageDropVolts = SystemHealthTelemetry.computeVoltageDrop(busVoltage);
      temperatureCelsius = intakePivot.getTemperature();

      deviceConnected = connectDebouncer.calculate(true);
      deviceFaultsRaw = intakePivot.getStickyFaultsRaw();

      if (deviceFaultsRaw != lastRawFaults) {
        decodedFaults = DeviceFaultDecoder.decode(DeviceType.TALON_FX, deviceFaultsRaw, 0);
        lastRawFaults = deviceFaultsRaw;
        if (decodedFaults.anyActive()) {
          faultTransitionCount++;
        }
      }
    } catch (Throwable t) {
      deviceConnected = connectDebouncer.calculate(false);
      deviceFaultsRaw = -1;
      decodedFaults = DeviceFaultDecoder.empty();
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
    busVoltage = 0;
    voltageDropVolts = 0;
    temperatureCelsius = 0;
  }

  @Override
  public void log() {
    SafeLog.put("IntakePivot/PositionRotations", positionRotations);
    SafeLog.put("IntakePivot/AtTarget", atTarget);
    SafeLog.put("IntakePivot/Device/Connected", deviceConnected);
    SafeLog.put("IntakePivot/CurrentAmps", currentAmps);
    SafeLog.put("IntakePivot/BusVoltage", busVoltage);
    SafeLog.put("IntakePivot/VoltageDropVolts", voltageDropVolts);
    SafeLog.put("IntakePivot/TemperatureCelsius", temperatureCelsius);
    SafeLog.put("IntakePivot/AppliedOutput", appliedOutput);
    DeviceFaultDecoder.publish(
        "IntakePivot", decodedFaults, deviceFaultsRaw, 0, faultTransitionCount);

    if (Constants.TUNING_MODE) {
      SafeLog.put("IntakePivot/Available", subsystemAvailable);
      SafeLog.put("IntakePivot/TargetPosition", targetPosition);
      SafeLog.put("IntakePivot/ActiveCommand", activeCommandName);
    }
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
