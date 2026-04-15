package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.util.DeviceFaultDecoder.DecodedFaults;
import frc.robot.util.DeviceFaultDecoder.DeviceType;
import org.junit.jupiter.api.Test;

class DeviceFaultDecoderTest {

  @Test
  void emptyInputReturnsEmptyRecord() {
    DecodedFaults result = DeviceFaultDecoder.decodeRevlib(0, 0);
    assertSame(DeviceFaultDecoder.empty(), result);
    assertFalse(result.anyActive());
    assertFalse(result.critical());
    assertEquals(0, result.activeBitCount());
    assertEquals("", result.shortSummary());
  }

  @Test
  void brownoutWarningBitDecodesToBrownoutAndCritical() {
    DecodedFaults result = DeviceFaultDecoder.decodeRevlib(0, 0x01);
    assertTrue(result.brownout());
    assertTrue(result.critical());
    assertEquals(1, result.activeBitCount());
    assertTrue(result.shortSummary().contains("brownout"));
  }

  @Test
  void gateDriverFaultBitDecodesToHardwareFaultAndCritical() {
    DecodedFaults result = DeviceFaultDecoder.decodeRevlib(0x20, 0);
    assertTrue(result.hardwareFault());
    assertTrue(result.critical());
    assertEquals(1, result.activeBitCount());
    assertTrue(result.shortSummary().contains("gate driver"));
  }

  @Test
  void stormOfFaultsDecodesAllExpectedFlags() {
    // Faults: motorType(0x02) + sensor(0x04) + gateDriver(0x20) + firmware(0x80) = 0xA6
    // Warnings: brownout(0x01) + overcurrent(0x02) + stall(0x20) + hasReset(0x40) = 0x63
    int faults = 0xA6;
    int warnings = 0x63;
    DecodedFaults result = DeviceFaultDecoder.decodeRevlib(faults, warnings);

    assertTrue(result.motorType());
    assertTrue(result.sensorFault());
    assertTrue(result.hardwareFault());
    assertTrue(result.firmware());
    assertTrue(result.brownout());
    assertTrue(result.overcurrent());
    assertTrue(result.stall());
    assertTrue(result.hasReset());
    assertTrue(result.critical());
    assertEquals(8, result.activeBitCount());

    // "gate driver" should appear early in the summary per ordering contract.
    String summary = result.shortSummary();
    int gateIdx = summary.indexOf("gate driver");
    int stallIdx = summary.indexOf("stall");
    assertTrue(gateIdx >= 0 && stallIdx >= 0);
    assertTrue(gateIdx < stallIdx, "gate driver should come before stall in summary");
  }

  @Test
  void activeBitCountMatchesPopCount() {
    // 4 fault bits + 3 warning bits = 7
    DecodedFaults result = DeviceFaultDecoder.decodeRevlib(0xAA, 0x15);
    assertEquals(Integer.bitCount(0xAA) + Integer.bitCount(0x15), result.activeBitCount());
  }

  @Test
  void summaryIsDeterministic() {
    DecodedFaults a = DeviceFaultDecoder.decodeRevlib(0x20, 0x02);
    DecodedFaults b = DeviceFaultDecoder.decodeRevlib(0x20, 0x02);
    assertEquals(a.shortSummary(), b.shortSummary());
    assertEquals(a, b);
  }

  @Test
  void summaryStaysUnderEightyCharacters() {
    // Every REVLib bit set.
    DecodedFaults result = DeviceFaultDecoder.decodeRevlib(0xFF, 0xFF);
    assertTrue(
        result.shortSummary().length() <= 80,
        "summary length " + result.shortSummary().length() + " exceeds 80");
  }

  @Test
  void temperatureWarningDoesNotMarkCritical() {
    // overtemp on its own is a watch condition, not a fix-now condition
    DecodedFaults result = DeviceFaultDecoder.decodeRevlib(0x10, 0);
    assertTrue(result.temperature());
    assertFalse(result.critical());
  }

  @Test
  void talonFxHardwareBitDecodesToHardwareFaultAndCritical() {
    int raw = 1 << 0; // hardware
    DecodedFaults result = DeviceFaultDecoder.decodeTalonFx(raw);
    assertTrue(result.hardwareFault());
    assertTrue(result.critical());
    assertEquals(1, result.activeBitCount());
  }

  @Test
  void talonFxBrownoutAndStatorLimitDecodesCorrectly() {
    int raw = (1 << 6) | (1 << 20); // bridge brownout + stator limit
    DecodedFaults result = DeviceFaultDecoder.decodeTalonFx(raw);
    assertTrue(result.brownout());
    assertTrue(result.overcurrent());
    assertTrue(result.critical());
    assertEquals(2, result.activeBitCount());
  }

  @Test
  void dispatcherRoutesToCorrectDecoder() {
    DecodedFaults sparkResult =
        DeviceFaultDecoder.decode(DeviceType.SPARK_FLEX, 0x20, 0); // gate driver
    assertTrue(sparkResult.hardwareFault());

    DecodedFaults talonResult =
        DeviceFaultDecoder.decode(DeviceType.TALON_FX, 1 << 3, 0); // undervoltage
    assertTrue(talonResult.underVoltage());
  }

  @Test
  void emptyAccessorReturnsNonNullSingleton() {
    assertNotNull(DeviceFaultDecoder.empty());
    assertSame(DeviceFaultDecoder.empty(), DeviceFaultDecoder.empty());
  }
}
