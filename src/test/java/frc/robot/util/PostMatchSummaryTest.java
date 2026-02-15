package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import java.lang.reflect.Field;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PostMatchSummaryTest {

  private PostMatchSummary summary;

  @BeforeEach
  void setUp() {
    HAL.initialize(500, 0);
    summary = PostMatchSummary.getInstance();
    // Reset all scoring fields to known state
    try {
      setField("isTracking", false);
      setField("brownoutCount", 0);
      setField("minBatteryVoltage", 13.0);
      setField("maxMotorTempCelsius", 30.0);
      setField("loopOverrunCount", 0);
    } catch (Exception e) {
      fail("Failed to reset PostMatchSummary state: " + e.getMessage());
    }
  }

  private void setField(String name, Object value) throws Exception {
    Field f = PostMatchSummary.class.getDeclaredField(name);
    f.setAccessible(true);
    f.set(summary, value);
  }

  @Test
  void testPerfectHealthScore() {
    assertEquals(100, summary.getLastHealthScore());
  }

  @Test
  void testOneBrownoutPenalty() throws Exception {
    setField("brownoutCount", 1);
    // 100 - 20 = 80
    assertEquals(80, summary.getLastHealthScore());
  }

  @Test
  void testTwoBrownoutsPenalty() throws Exception {
    setField("brownoutCount", 2);
    // 100 - 40 = 60
    assertEquals(60, summary.getLastHealthScore());
  }

  @Test
  void testThreeBrownoutsStackPenalty() throws Exception {
    setField("brownoutCount", 3);
    // 100 - 60 = 40
    assertEquals(40, summary.getLastHealthScore());
  }

  @Test
  void testLowBatteryPenalty() throws Exception {
    setField("minBatteryVoltage", 10.5);
    // 100 - 10 = 90
    assertEquals(90, summary.getLastHealthScore());
  }

  @Test
  void testBatteryAt11VNoPenalty() throws Exception {
    // 11.0 < 11.0 is false, no penalty
    setField("minBatteryVoltage", 11.0);
    assertEquals(100, summary.getLastHealthScore());
  }

  @Test
  void testBatteryAbove11NoPenalty() throws Exception {
    setField("minBatteryVoltage", 12.5);
    assertEquals(100, summary.getLastHealthScore());
  }

  @Test
  void testHighTempPenalty() throws Exception {
    setField("maxMotorTempCelsius", 70.0);
    // 100 - 5 = 95
    assertEquals(95, summary.getLastHealthScore());
  }

  @Test
  void testTempAt65NoPenalty() throws Exception {
    // 65.0 > 65.0 is false, no penalty
    setField("maxMotorTempCelsius", 65.0);
    assertEquals(100, summary.getLastHealthScore());
  }

  @Test
  void testTempBelow65NoPenalty() throws Exception {
    setField("maxMotorTempCelsius", 50.0);
    assertEquals(100, summary.getLastHealthScore());
  }

  @Test
  void testLoopOverrunPenalty() throws Exception {
    setField("loopOverrunCount", 30);
    // 30 / 10 = 3 penalty points -> 100 - 3 = 97
    assertEquals(97, summary.getLastHealthScore());
  }

  @Test
  void testExactly10OverrunsOnePenalty() throws Exception {
    setField("loopOverrunCount", 10);
    // 10 / 10 = 1 -> 100 - 1 = 99
    assertEquals(99, summary.getLastHealthScore());
  }

  @Test
  void testFewOverrunsNoPenalty() throws Exception {
    setField("loopOverrunCount", 5);
    // 5 / 10 = 0 (integer division) -> 100
    assertEquals(100, summary.getLastHealthScore());
  }

  @Test
  void testMultiplePenaltiesStack() throws Exception {
    setField("brownoutCount", 1); // -20
    setField("minBatteryVoltage", 10.0); // -10
    setField("maxMotorTempCelsius", 75.0); // -5
    setField("loopOverrunCount", 20); // -2
    // 100 - 20 - 10 - 5 - 2 = 63
    assertEquals(63, summary.getLastHealthScore());
  }

  @Test
  void testScoreClampedToZero() throws Exception {
    setField("brownoutCount", 10); // -200, but clamped to 0
    assertEquals(0, summary.getLastHealthScore());
  }

  @Test
  void testScoreNeverNegative() throws Exception {
    setField("brownoutCount", 6); // -120
    setField("minBatteryVoltage", 8.0); // -10
    setField("maxMotorTempCelsius", 90.0); // -5
    setField("loopOverrunCount", 500); // -50
    assertEquals(0, summary.getLastHealthScore());
  }

  @Test
  void testGetMinBatteryVoltage() throws Exception {
    setField("minBatteryVoltage", 11.2);
    assertEquals(11.2, summary.getMinBatteryVoltage(), 0.01);
  }

  @Test
  void testGetMaxMotorTemp() throws Exception {
    setField("maxMotorTempCelsius", 55.0);
    assertEquals(55.0, summary.getMaxMotorTemp(), 0.01);
  }

  @Test
  void testIsTrackingDefault() throws Exception {
    setField("isTracking", false);
    assertFalse(summary.isTracking());
  }

  @Test
  void testZeroBatteryVoltage() throws Exception {
    setField("minBatteryVoltage", 0.0);
    // Should still compute: 100 - 10 (low battery) = 90
    assertEquals(90, summary.getLastHealthScore());
  }

  @Test
  void testExtremeTemperature() throws Exception {
    setField("maxMotorTempCelsius", 200.0);
    // Still just -5 penalty (single threshold, not scaled)
    assertEquals(95, summary.getLastHealthScore());
  }

  @Test
  void testMaxOverruns() throws Exception {
    setField("loopOverrunCount", 10000);
    // 10000 / 10 = 1000 penalty -> clamped to 0
    assertEquals(0, summary.getLastHealthScore());
  }
}
