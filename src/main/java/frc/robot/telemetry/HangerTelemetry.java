package frc.robot.telemetry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DeviceHealthConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.Hanger;
import frc.robot.util.DeviceFaultDecoder;
import frc.robot.util.DeviceFaultDecoder.DecodedFaults;
import frc.robot.util.DeviceFaultDecoder.DeviceType;
import frc.robot.util.SafeLog;

/** Hanger telemetry: position, deploy/climb state, motor health. */
class HangerTelemetry implements SubsystemTelemetry {
  private Hanger hanger; // grabbed again in update() if not ready yet
  private boolean subsystemAvailable = false;

  private double position = 0;
  private double targetPosition = 0;
  private double positionError = 0;
  private boolean atPosition = false;
  private double appliedOutput = 0;
  private double currentAmps = 0;
  private double busVoltage = 0;
  private double voltageDropVolts = 0;
  private double temperatureCelsius = 0;
  private boolean isDeployed = false;
  private boolean isClimbing = false;

  private final Debouncer connectDebouncer =
      new Debouncer(DeviceHealthConstants.DISCONNECT_DEBOUNCE_SEC, Debouncer.DebounceType.kFalling);
  private boolean deviceConnected = false;
  private int deviceFaultsRaw = 0;
  private int deviceWarningsRaw = 0;
  private int lastRawFaults = 0;
  private int lastRawWarnings = 0;
  private int faultTransitionCount = 0;
  private DecodedFaults decodedFaults = DeviceFaultDecoder.empty();

  private double climbProgress = 0;
  private String controlMode = "UNKNOWN";
  private boolean softLimitActive = false;

  private String activeCommandName = "none";

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
      busVoltage = hanger.getBusVoltage();
      voltageDropVolts = SystemHealthTelemetry.computeVoltageDrop(busVoltage);
      temperatureCelsius = hanger.getTemperature();

      isDeployed =
          Math.abs(position - MotorConstants.UP_HANGER_POS) < MotorConstants.HANGER_POS_TOLERANCE;
      isClimbing = targetPosition == MotorConstants.DOWN_HANGER_POS && !atPosition;

      double range = MotorConstants.UP_HANGER_POS - MotorConstants.DOWN_HANGER_POS;
      climbProgress =
          (range > 0)
              ? MathUtil.clamp((position - MotorConstants.DOWN_HANGER_POS) / range, 0, 1)
              : 0;

      deviceConnected = connectDebouncer.calculate(true);
      deviceFaultsRaw = hanger.getStickyFaultsRaw();
      deviceWarningsRaw = hanger.getStickyWarningsRaw();

      // Only re-decode when the raw integers change so the hot loop stays allocation-light
      // during a clean match.
      if (deviceFaultsRaw != lastRawFaults || deviceWarningsRaw != lastRawWarnings) {
        decodedFaults =
            DeviceFaultDecoder.decode(DeviceType.SPARK_MAX, deviceFaultsRaw, deviceWarningsRaw);
        lastRawFaults = deviceFaultsRaw;
        lastRawWarnings = deviceWarningsRaw;
        if (decodedFaults.anyActive()) {
          faultTransitionCount++;
        }
      }

      controlMode = (targetPosition != 0) ? "POSITION" : "OPEN_LOOP";
      softLimitActive =
          (position <= MotorConstants.DOWN_HANGER_POS + 0.1)
              || (position >= MotorConstants.UP_HANGER_POS - 0.1);
    } catch (Throwable t) {
      deviceConnected = connectDebouncer.calculate(false);
      deviceFaultsRaw = -1;
      deviceWarningsRaw = -1;
      decodedFaults = DeviceFaultDecoder.empty();
      setDefaultValues();
    }

    try {
      Command currentCmd = hanger.getCurrentCommand();
      activeCommandName = (currentCmd != null) ? currentCmd.getName() : "none";
    } catch (Throwable t) {
      activeCommandName = "unknown";
    }
  }

  private void setDefaultValues() {
    position = 0;
    targetPosition = 0;
    positionError = 0;
    atPosition = false;
    appliedOutput = 0;
    currentAmps = 0;
    busVoltage = 0;
    voltageDropVolts = 0;
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
    SafeLog.put("Hanger/BusVoltage", busVoltage);
    SafeLog.put("Hanger/VoltageDropVolts", voltageDropVolts);
    SafeLog.put("Hanger/TemperatureCelsius", temperatureCelsius);
    SafeLog.put("Hanger/IsDeployed", isDeployed);
    SafeLog.put("Hanger/IsClimbing", isClimbing);

    SafeLog.put("Hanger/Device/Connected", deviceConnected);
    DeviceFaultDecoder.publish(
        "Hanger", decodedFaults, deviceFaultsRaw, deviceWarningsRaw, faultTransitionCount);
    SafeLog.put("Hanger/ClimbProgress", climbProgress);
    SafeLog.put("Hanger/ControlMode", controlMode);
    SafeLog.put("Hanger/SoftLimitActive", softLimitActive);
    SafeLog.put("Hanger/ActiveCommand", activeCommandName);
  }

  @Override
  public String getName() {
    return "Hanger";
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
