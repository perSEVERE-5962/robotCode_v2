package frc.robot.telemetry;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DeviceHealthConstants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.util.DeviceFaultDecoder;
import frc.robot.util.DeviceFaultDecoder.DecodedFaults;
import frc.robot.util.DeviceFaultDecoder.DeviceType;

/** IntakeActuator telemetry: position and motor health. */
public class IntakeActuatorTelemetry implements SubsystemTelemetry {
  private IntakePivot intakeActuator; // grabbed again in update() if not ready yet
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

  public IntakeActuatorTelemetry() {
    this.intakeActuator = IntakePivot.getInstance();
  }

  @Override
  public void update() {
    if (intakeActuator == null) {
      intakeActuator = IntakePivot.getInstance();
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
      busVoltage = intakeActuator.getBusVoltage();
      voltageDropVolts = SystemHealthTelemetry.computeVoltageDrop(busVoltage);
      temperatureCelsius = intakeActuator.getTemperature();

      deviceConnected = connectDebouncer.calculate(true);
      deviceFaultsRaw = intakeActuator.getStickyFaultsRaw();

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
      Command currentCmd = intakeActuator.getCurrentCommand();
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
    SafeLog.put("IntakeActuator/PositionRotations", positionRotations);
    SafeLog.put("IntakeActuator/AtTarget", atTarget);
    SafeLog.put("IntakeActuator/Device/Connected", deviceConnected);
    SafeLog.put("IntakeActuator/CurrentAmps", currentAmps);
    SafeLog.put("IntakeActuator/BusVoltage", busVoltage);
    SafeLog.put("IntakeActuator/VoltageDropVolts", voltageDropVolts);
    SafeLog.put("IntakeActuator/TemperatureCelsius", temperatureCelsius);
    SafeLog.put("IntakeActuator/AppliedOutput", appliedOutput);
    DeviceFaultDecoder.publish(
        "IntakeActuator", decodedFaults, deviceFaultsRaw, 0, faultTransitionCount);

    if (Constants.TUNING_MODE) {
      SafeLog.put("IntakeActuator/Available", subsystemAvailable);
      SafeLog.put("IntakeActuator/TargetPosition", targetPosition);
      SafeLog.put("IntakeActuator/ActiveCommand", activeCommandName);
    }
  }

  @Override
  public String getName() {
    return "IntakeActuator";
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
