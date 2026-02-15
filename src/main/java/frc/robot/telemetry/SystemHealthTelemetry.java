package frc.robot.telemetry;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

/** System health: battery, CAN bus, roboRIO, loop timing, brownout prediction. */
public class SystemHealthTelemetry implements SubsystemTelemetry {
  // M4: Increased threshold to reduce false positives from scheduler jitter
  private static final double LOOP_OVERRUN_THRESHOLD_MS = 25.0;

  // L4: Decimation for CANStatus to reduce GC pressure
  private static final int CAN_STATUS_DECIMATION = 10;
  private int cycleCounter = 0;

  // Current state
  private double batteryVoltage = 0;
  private double canUtilization = 0;
  private int canTxErrors = 0;
  private int canRxErrors = 0;
  private int canBusOff = 0;
  private double rioCPUTemp = 0;
  private double rio3V3Rail = 0;
  private double rio5VRail = 0;
  private double rio6VRail = 0;
  private boolean brownedOut = false;
  private boolean rslState = false;

  // Additional voltage signals
  private double brownoutVoltage = 0;
  private double inputVoltage = 0;

  private double lastVoltage = 0;
  private double lastVoltageTimestamp = 0;
  private double voltageSlope = 0; // V/s, negative = dropping
  private int brownoutRiskLevel = 0; // 0=none, 1=watch, 2=warning, 3=imminent
  private boolean brownoutRisk = false;

  // Total current draw from PDH
  private PowerDistribution pdh = null;
  private double totalCurrentAmps = 0;
  private double peakCurrentAmps = 0;

  // Loop timing (computed here since we're called every cycle)
  private double lastLoopTimestamp = 0;
  private double loopTimeMs = 0;
  private int loopOverrunCount = 0;

  public SystemHealthTelemetry() {
    try {
      pdh = new PowerDistribution();
    } catch (Throwable t) {
      // PDH not available (sim or missing hardware)
      pdh = null;
    }
  }

  @Override
  public void update() {
    // Loop timing - must be first to measure time since last cycle
    // This section must ALWAYS run, never guarded
    double currentTimestamp = Timer.getFPGATimestamp();
    if (lastLoopTimestamp > 0) {
      loopTimeMs = (currentTimestamp - lastLoopTimestamp) * 1000.0;
      if (loopTimeMs > LOOP_OVERRUN_THRESHOLD_MS) { // M4
        loopOverrunCount++;
      }
    }
    lastLoopTimestamp = currentTimestamp;

    // M8: Guard HAL reads - preserve loop timing even if HAL fails
    try {
      // Battery and power
      batteryVoltage = RobotController.getBatteryVoltage();

      double now = Timer.getFPGATimestamp();
      if (lastVoltageTimestamp > 0 && lastVoltage > 0) {
        double deltaTime = now - lastVoltageTimestamp;
        if (deltaTime > 0.001) { // Avoid divide-by-zero
          voltageSlope = (batteryVoltage - lastVoltage) / deltaTime;
        }
      }
      lastVoltage = batteryVoltage;
      lastVoltageTimestamp = now;

      // Level 0: voltage > 10V OR slope > -1 V/s
      // Level 1: voltage < 10V AND slope < -1 V/s
      // Level 2: voltage < 9V AND slope < -1.5 V/s
      // Level 3: voltage < 8V AND slope < -2 V/s
      if (batteryVoltage < 8.0 && voltageSlope < -2.0) {
        brownoutRiskLevel = 3;
      } else if (batteryVoltage < 9.0 && voltageSlope < -1.5) {
        brownoutRiskLevel = 2;
      } else if (batteryVoltage < 10.0 && voltageSlope < -1.0) {
        brownoutRiskLevel = 1;
      } else {
        brownoutRiskLevel = 0;
      }
      brownoutRisk = (brownoutRiskLevel >= 2);

      // L4: Decimate CANStatus reads to every 10 cycles
      cycleCounter++;
      if (cycleCounter >= CAN_STATUS_DECIMATION) {
        cycleCounter = 0;
        CANStatus canStatus = RobotController.getCANStatus();
        canUtilization = canStatus.percentBusUtilization;
        canTxErrors = canStatus.transmitErrorCount;
        canRxErrors = canStatus.receiveErrorCount;
        canBusOff = canStatus.busOffCount;
      }

      // roboRIO
      rioCPUTemp = RobotController.getCPUTemp();
      rio3V3Rail = RobotController.getVoltage3V3();
      rio5VRail = RobotController.getVoltage5V();
      rio6VRail = RobotController.getVoltage6V();
      brownedOut = RobotController.isBrownedOut();
      rslState = RobotController.getRSLState();

      // Additional voltage signals
      brownoutVoltage = RobotController.getBrownoutVoltage();
      inputVoltage = RobotController.getInputVoltage();

      // Total current draw from PDH
      if (pdh != null) {
        totalCurrentAmps = pdh.getTotalCurrent();
        if (totalCurrentAmps > peakCurrentAmps) {
          peakCurrentAmps = totalCurrentAmps;
        }
      }
    } catch (Throwable t) {
      batteryVoltage = 0;
      canUtilization = 0;
      rioCPUTemp = 0;
      brownedOut = false;
      brownoutVoltage = 0;
      inputVoltage = 0;
      voltageSlope = 0;
      brownoutRiskLevel = 0;
      brownoutRisk = false;
      totalCurrentAmps = 0;
    }
  }

  @Override
  public void log() {
    SafeLog.put("SystemHealth/BatteryVoltage", batteryVoltage);
    SafeLog.put("SystemHealth/LoopTimeMs", loopTimeMs);
    SafeLog.put("SystemHealth/LoopOverruns", loopOverrunCount);
    SafeLog.put("SystemHealth/CANUtilization", canUtilization);
    SafeLog.put("SystemHealth/CANTxErrors", canTxErrors);
    SafeLog.put("SystemHealth/CANRxErrors", canRxErrors);
    SafeLog.put("SystemHealth/CANBusOff", canBusOff);
    SafeLog.put("SystemHealth/RioCPUTemp", rioCPUTemp);
    SafeLog.put("SystemHealth/Rio3V3Rail", rio3V3Rail);
    SafeLog.put("SystemHealth/Rio5VRail", rio5VRail);
    SafeLog.put("SystemHealth/Rio6VRail", rio6VRail);
    SafeLog.put("SystemHealth/BrownedOut", brownedOut);
    SafeLog.put("SystemHealth/RSLState", rslState);

    // Additional voltage signals
    SafeLog.put("SystemHealth/BrownoutVoltage", brownoutVoltage);
    SafeLog.put("SystemHealth/InputVoltage", inputVoltage);

    SafeLog.put("SystemHealth/VoltageSlope", voltageSlope);
    SafeLog.put("SystemHealth/BrownoutRisk", brownoutRisk);
    SafeLog.put("SystemHealth/BrownoutRiskLevel", brownoutRiskLevel);

    SafeLog.put("SystemHealth/TotalCurrentAmps", totalCurrentAmps);
    SafeLog.put("SystemHealth/PeakCurrentAmps", peakCurrentAmps);
  }

  @Override
  public String getName() {
    return "SystemHealth";
  }

  // Accessors for AlertManager
  public double getLoopTimeMs() {
    return loopTimeMs;
  }

  public int getLoopOverrunCount() {
    return loopOverrunCount;
  }

  public double getVoltageSlope() {
    return voltageSlope;
  }

  public int getBrownoutRiskLevel() {
    return brownoutRiskLevel;
  }

  public boolean isBrownoutRisk() {
    return brownoutRisk;
  }

  public double getTotalCurrentAmps() {
    return totalCurrentAmps;
  }

  public double getPeakCurrentAmps() {
    return peakCurrentAmps;
  }
}
