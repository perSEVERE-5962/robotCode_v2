package frc.robot.util;

import frc.robot.telemetry.SafeLog;

/**
 * The pit crew and the electrical team cannot memorize a fault bit table under match pressure. A
 * raw fault integer like {@code 0x0024} tells them nothing in the five minutes they have between
 * matches. Sharing one place that turns vendor fault words into the phrase "gate driver, stall"
 * lets every dashboard tile, every alert rule, and every telemetry class produce the same human
 * readable output without each re-implementing bit masks that change when the vendor updates the
 * firmware.
 *
 * <p>Pure and stateless so unit tests run in plain JUnit without sim fixtures. Allocation aware so
 * the hot loop in each telemetry class can skip the decode step when the raw integer has not
 * changed from last cycle.
 */
public final class DeviceFaultDecoder {

  /** Which device family produced the raw integer. Controls how bits are interpreted. */
  public enum DeviceType {
    SPARK_MAX,
    SPARK_FLEX,
    TALON_FX
  }

  /**
   * Union of the flags that matter across both vendor families. Fields that do not apply to a given
   * device type stay false.
   *
   * <p>{@code hardwareFault} is the "replace the controller" flag (gate driver on REVLib, generic
   * hardware on TalonFX). {@code critical} is the pit-crew rollup: true when brownout, overcurrent,
   * hardwareFault, motorType, firmware, or a hardware-level sensor fault is set. {@code
   * shortSummary} is the comma separated human readable list, capped at 80 characters so it fits
   * inside an Elastic tile.
   */
  public record DecodedFaults(
      boolean brownout,
      boolean overcurrent,
      boolean underVoltage,
      boolean overVoltage,
      boolean temperature,
      boolean hardwareFault,
      boolean sensorFault,
      boolean motorType,
      boolean firmware,
      boolean eeprom,
      boolean stall,
      boolean hasReset,
      boolean canError,
      boolean bootDuringEnable,
      int activeBitCount,
      boolean anyActive,
      boolean critical,
      String shortSummary) {}

  private static final DecodedFaults EMPTY =
      new DecodedFaults(
          false, false, false, false, false, false, false, false, false, false, false, false, false,
          false, 0, false, false, "");

  private DeviceFaultDecoder() {}

  /** Returns a singleton empty record for use as a default when the device is unavailable. */
  public static DecodedFaults empty() {
    return EMPTY;
  }

  /**
   * Decodes the REVLib fault and warning raw bit words into a DecodedFaults record. Fault bits and
   * warning bits come from separate accessors on the underlying SparkBase, so both are passed in.
   *
   * <p>Bit positions come from {@code SparkBase.Faults} and {@code SparkBase.Warnings} in the 2026
   * REVLib source.
   *
   * <p><b>Faults (hardware level, 8 bits):</b> other, motorType, sensor, can, temperature,
   * gateDriver, escEeprom, firmware.
   *
   * <p><b>Warnings (operational level, 8 bits):</b> brownout, overcurrent, escEeprom, extEeprom,
   * sensor, stall, hasReset, other.
   */
  public static DecodedFaults decodeRevlib(int faultsRaw, int warningsRaw) {
    if (faultsRaw == 0 && warningsRaw == 0) {
      return EMPTY;
    }

    boolean motorType = (faultsRaw & 0x02) != 0;
    boolean faultSensor = (faultsRaw & 0x04) != 0;
    boolean faultCan = (faultsRaw & 0x08) != 0;
    boolean faultTemperature = (faultsRaw & 0x10) != 0;
    boolean gateDriver = (faultsRaw & 0x20) != 0;
    boolean faultEeprom = (faultsRaw & 0x40) != 0;
    boolean firmware = (faultsRaw & 0x80) != 0;

    boolean brownout = (warningsRaw & 0x01) != 0;
    boolean overcurrent = (warningsRaw & 0x02) != 0;
    boolean warnEscEeprom = (warningsRaw & 0x04) != 0;
    boolean warnExtEeprom = (warningsRaw & 0x08) != 0;
    boolean warnSensor = (warningsRaw & 0x10) != 0;
    boolean stall = (warningsRaw & 0x20) != 0;
    boolean hasReset = (warningsRaw & 0x40) != 0;

    boolean sensorFault = faultSensor || warnSensor;
    boolean eeprom = faultEeprom || warnEscEeprom || warnExtEeprom;
    boolean hardwareFault = gateDriver;

    int bitCount = Integer.bitCount(faultsRaw & 0xFF) + Integer.bitCount(warningsRaw & 0xFF);

    boolean critical =
        brownout
            || overcurrent
            || hardwareFault
            || motorType
            || firmware
            || (faultSensor && !warnSensor); // hardware sensor fault, not just a warning

    String summary =
        buildSummary(
            brownout,
            overcurrent,
            false,
            false,
            faultTemperature,
            hardwareFault,
            sensorFault,
            motorType,
            firmware,
            eeprom,
            stall,
            hasReset,
            faultCan,
            false);

    return new DecodedFaults(
        brownout,
        overcurrent,
        false,
        false,
        faultTemperature,
        hardwareFault,
        sensorFault,
        motorType,
        firmware,
        eeprom,
        stall,
        hasReset,
        faultCan,
        false,
        bitCount,
        true,
        critical,
        summary);
  }

  /**
   * Decodes the Phoenix6 TalonFX {@code getFaultField()} bitfield. Bit positions follow the
   * Phoenix6 2026 fault enum.
   *
   * <p>Only the electrical and safety bits that matter to the pit crew are mapped; limit-switch and
   * soft-limit bits are not surfaced because they are part of normal operation and would flood the
   * dashboard during every pivot move.
   */
  public static DecodedFaults decodeTalonFx(int faultsRaw) {
    if (faultsRaw == 0) {
      return EMPTY;
    }

    boolean hardware = (faultsRaw & (1 << 0)) != 0;
    boolean procTemp = (faultsRaw & (1 << 1)) != 0;
    boolean deviceTemp = (faultsRaw & (1 << 2)) != 0;
    boolean underVoltage = (faultsRaw & (1 << 3)) != 0;
    boolean bootDuringEnable = (faultsRaw & (1 << 4)) != 0;
    boolean bridgeBrownout = (faultsRaw & (1 << 6)) != 0;
    boolean remoteSensorReset = (faultsRaw & (1 << 7)) != 0;
    boolean overSupplyV = (faultsRaw & (1 << 10)) != 0;
    boolean unstableSupplyV = (faultsRaw & (1 << 11)) != 0;
    boolean remoteSensorInvalid = (faultsRaw & (1 << 18)) != 0;
    boolean statorLimit = (faultsRaw & (1 << 20)) != 0;

    boolean temperature = procTemp || deviceTemp;
    boolean hardwareFault = hardware;
    boolean sensorFault = remoteSensorReset || remoteSensorInvalid;
    boolean brownout = bridgeBrownout;
    boolean overcurrent = statorLimit;

    int bitCount = Integer.bitCount(faultsRaw);

    boolean critical =
        hardwareFault || brownout || underVoltage || overcurrent || bootDuringEnable || overSupplyV;

    String summary =
        buildSummary(
            brownout,
            overcurrent,
            underVoltage,
            overSupplyV || unstableSupplyV,
            temperature,
            hardwareFault,
            sensorFault,
            false,
            false,
            false,
            false,
            false,
            false,
            bootDuringEnable);

    return new DecodedFaults(
        brownout,
        overcurrent,
        underVoltage,
        overSupplyV || unstableSupplyV,
        temperature,
        hardwareFault,
        sensorFault,
        false,
        false,
        false,
        false,
        false,
        false,
        bootDuringEnable,
        bitCount,
        true,
        critical,
        summary);
  }

  /** REVLib needs the two-integer form; TalonFX reads only {@code faultsRaw}. */
  public static DecodedFaults decode(DeviceType type, int faultsRaw, int warningsRaw) {
    return switch (type) {
      case SPARK_MAX, SPARK_FLEX -> decodeRevlib(faultsRaw, warningsRaw);
      case TALON_FX -> decodeTalonFx(faultsRaw);
    };
  }

  /**
   * Order is load-bearing: the most actionable items come first so tile truncation still leaves the
   * replace-the-controller keywords visible.
   */
  private static String buildSummary(
      boolean brownout,
      boolean overcurrent,
      boolean underVoltage,
      boolean overVoltage,
      boolean temperature,
      boolean hardwareFault,
      boolean sensorFault,
      boolean motorType,
      boolean firmware,
      boolean eeprom,
      boolean stall,
      boolean hasReset,
      boolean canError,
      boolean bootDuringEnable) {
    StringBuilder sb = new StringBuilder(80);
    appendIf(sb, hardwareFault, "gate driver");
    appendIf(sb, motorType, "motor type");
    appendIf(sb, firmware, "firmware lock");
    appendIf(sb, sensorFault, "sensor");
    appendIf(sb, brownout, "brownout");
    appendIf(sb, overcurrent, "overcurrent");
    appendIf(sb, underVoltage, "undervoltage");
    appendIf(sb, overVoltage, "oversupply");
    appendIf(sb, temperature, "overtemp");
    appendIf(sb, stall, "stall");
    appendIf(sb, eeprom, "eeprom");
    appendIf(sb, canError, "can error");
    appendIf(sb, hasReset, "reset");
    appendIf(sb, bootDuringEnable, "boot-on-enable");

    if (sb.length() > 80) {
      sb.setLength(77);
      sb.append("...");
    }
    return sb.toString();
  }

  /**
   * One call per subsystem in {@code log()} so every telemetry class produces the same signal
   * layout without repeating nineteen {@code SafeLog.put} calls by hand.
   */
  public static void publish(
      String prefix, DecodedFaults decoded, int faultsRaw, int warningsRaw, int transitionCount) {
    SafeLog.put(prefix + "/Device/FaultsRaw", faultsRaw);
    SafeLog.put(prefix + "/Device/WarningsRaw", warningsRaw);
    SafeLog.put(prefix + "/Fault/Brownout", decoded.brownout());
    SafeLog.put(prefix + "/Fault/Overcurrent", decoded.overcurrent());
    SafeLog.put(prefix + "/Fault/UnderVoltage", decoded.underVoltage());
    SafeLog.put(prefix + "/Fault/OverVoltage", decoded.overVoltage());
    SafeLog.put(prefix + "/Fault/Temperature", decoded.temperature());
    SafeLog.put(prefix + "/Fault/Hardware", decoded.hardwareFault());
    SafeLog.put(prefix + "/Fault/Sensor", decoded.sensorFault());
    SafeLog.put(prefix + "/Fault/MotorType", decoded.motorType());
    SafeLog.put(prefix + "/Fault/Firmware", decoded.firmware());
    SafeLog.put(prefix + "/Fault/Eeprom", decoded.eeprom());
    SafeLog.put(prefix + "/Fault/Stall", decoded.stall());
    SafeLog.put(prefix + "/Fault/HasReset", decoded.hasReset());
    SafeLog.put(prefix + "/Fault/CanError", decoded.canError());
    SafeLog.put(prefix + "/Fault/BootDuringEnable", decoded.bootDuringEnable());
    SafeLog.put(prefix + "/Fault/ActiveBits", decoded.activeBitCount());
    SafeLog.put(prefix + "/Fault/Critical", decoded.critical());
    SafeLog.put(prefix + "/Fault/Summary", decoded.shortSummary());
    SafeLog.put(prefix + "/Fault/TransitionCount", transitionCount);
  }

  private static void appendIf(StringBuilder sb, boolean flag, String label) {
    if (!flag) return;
    if (sb.length() > 0) {
      sb.append(", ");
    }
    sb.append(label);
  }
}
