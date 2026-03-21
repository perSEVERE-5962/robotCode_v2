package frc.robot.telemetry;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.PDHChannelMap;

/** System health: battery, CAN bus, roboRIO, loop timing, brownout prediction. */
public class SystemHealthTelemetry implements SubsystemTelemetry {
  private static final double LOOP_OVERRUN_THRESHOLD_MS = 25.0;

  private static final int CAN_STATUS_DECIMATION = 10;
  private int cycleCounter = 0;

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

  private double brownoutVoltage = 0;
  private double inputVoltage = 0;

  private double lastVoltage = 0;
  private double lastVoltageTimestamp = 0;
  private double voltageSlope = 0; // V/s, negative = dropping (EMA-smoothed)
  private int brownoutRiskLevel = 0; // 0=none, 1=watch, 2=warning, 3=imminent
  private boolean brownoutRisk = false;

  // smooths out noisy voltage slope so brownout risk level doesn't flicker
  private static final double VOLTAGE_SLOPE_EMA_ALPHA = 0.3;

  private PowerDistribution pdh = null;
  private double totalCurrentAmps = 0;
  private double peakCurrentAmps = 0;

  private static final int PDH_CHANNEL_DECIMATION = 5;
  private int pdhChannelCycleCounter = 0;
  private final double[] channelCurrents = new double[PDHChannelMap.NUM_CHANNELS];
  private final double[] channelPeakCurrents = new double[PDHChannelMap.NUM_CHANNELS];
  private String overcurrentChannel = "none";
  private double overcurrentAmps = 0;
  private boolean overcurrentAlert = false;

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
    try {
      double currentTimestamp = Timer.getFPGATimestamp();
      if (lastLoopTimestamp > 0) {
        loopTimeMs = (currentTimestamp - lastLoopTimestamp) * 1000.0;
        if (loopTimeMs > LOOP_OVERRUN_THRESHOLD_MS) {
          loopOverrunCount++;
        }
      }
      lastLoopTimestamp = currentTimestamp;

      batteryVoltage = RobotController.getBatteryVoltage();

      double now = Timer.getFPGATimestamp();
      if (lastVoltageTimestamp > 0 && lastVoltage > 0) {
        double deltaTime = now - lastVoltageTimestamp;
        if (deltaTime > 0.001) {
          double rawSlope = (batteryVoltage - lastVoltage) / deltaTime;
          // EMA smoothing so brownout risk level doesn't flicker on noisy samples
          voltageSlope =
              VOLTAGE_SLOPE_EMA_ALPHA * rawSlope + (1.0 - VOLTAGE_SLOPE_EMA_ALPHA) * voltageSlope;
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

      cycleCounter++;
      if (cycleCounter >= CAN_STATUS_DECIMATION) {
        cycleCounter = 0;
        CANStatus canStatus = RobotController.getCANStatus();
        canUtilization = canStatus.percentBusUtilization;
        canTxErrors = canStatus.transmitErrorCount;
        canRxErrors = canStatus.receiveErrorCount;
        canBusOff = canStatus.busOffCount;
      }

      rioCPUTemp = RobotController.getCPUTemp();
      rio3V3Rail = RobotController.getVoltage3V3();
      rio5VRail = RobotController.getVoltage5V();
      rio6VRail = RobotController.getVoltage6V();
      brownedOut = RobotController.isBrownedOut();
      rslState = RobotController.getRSLState();

      brownoutVoltage = RobotController.getBrownoutVoltage();
      inputVoltage = RobotController.getInputVoltage();

      if (pdh != null) {
        totalCurrentAmps = pdh.getTotalCurrent();
        if (totalCurrentAmps > peakCurrentAmps) {
          peakCurrentAmps = totalCurrentAmps;
        }

        pdhChannelCycleCounter++;
        if (pdhChannelCycleCounter >= PDH_CHANNEL_DECIMATION) {
          pdhChannelCycleCounter = 0;
          double[] raw = pdh.getAllCurrents();
          int count = Math.min(raw.length, PDHChannelMap.NUM_CHANNELS);
          double maxCurrent = 0;
          int maxChannel = -1;

          for (int i = 0; i < count; i++) {
            channelCurrents[i] = raw[i];
            if (raw[i] > channelPeakCurrents[i]) {
              channelPeakCurrents[i] = raw[i];
            }
            if (raw[i] > maxCurrent) {
              maxCurrent = raw[i];
              maxChannel = i;
            }
          }

          if (maxCurrent >= PDHChannelMap.CHANNEL_OVERCURRENT_AMPS && maxChannel >= 0) {
            overcurrentAlert = true;
            overcurrentChannel = PDHChannelMap.getLabel(maxChannel);
            overcurrentAmps = maxCurrent;
          } else {
            overcurrentAlert = false;
            overcurrentChannel = "none";
            overcurrentAmps = 0;
          }
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

    SafeLog.put("SystemHealth/BrownoutVoltage", brownoutVoltage);
    SafeLog.put("SystemHealth/InputVoltage", inputVoltage);

    SafeLog.put("SystemHealth/VoltageSlope", voltageSlope);
    SafeLog.put("SystemHealth/BrownoutRisk", brownoutRisk);
    SafeLog.put("SystemHealth/BrownoutRiskLevel", brownoutRiskLevel);

    SafeLog.put("SystemHealth/TotalCurrentAmps", totalCurrentAmps);
    SafeLog.put("SystemHealth/PeakCurrentAmps", peakCurrentAmps);

    for (int i = 0; i < PDHChannelMap.NUM_CHANNELS; i++) {
      String label = PDHChannelMap.getLabel(i);
      SafeLog.put("PDH/" + label + "/Current", channelCurrents[i]);
      SafeLog.put("PDH/" + label + "/PeakCurrent", channelPeakCurrents[i]);
    }

    SafeLog.put("PDH/OvercurrentAlert", overcurrentAlert);
    SafeLog.put("PDH/OvercurrentChannel", overcurrentChannel);
    SafeLog.put("PDH/OvercurrentAmps", overcurrentAmps);
  }

  @Override
  public String getName() {
    return "SystemHealth";
  }

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

  public boolean isOvercurrentAlert() {
    return overcurrentAlert;
  }

  public String getOvercurrentChannel() {
    return overcurrentChannel;
  }

  public double getChannelCurrent(int channel) {
    if (channel >= 0 && channel < channelCurrents.length) {
      return channelCurrents[channel];
    }
    return 0;
  }
}
