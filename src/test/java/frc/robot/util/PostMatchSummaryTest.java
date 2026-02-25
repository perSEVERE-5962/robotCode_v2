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
    assertEquals(80, summary.getLastHealthScore());
  }

  @Test
  void testMultiplePenaltiesStack() throws Exception {
    setField("brownoutCount", 1);
    setField("minBatteryVoltage", 10.0);
    setField("maxMotorTempCelsius", 75.0);
    setField("loopOverrunCount", 20);
    assertEquals(63, summary.getLastHealthScore());
  }

  @Test
  void testScoreClampedToZero() throws Exception {
    setField("brownoutCount", 10);
    assertEquals(0, summary.getLastHealthScore());
  }

  @Test
  void testLowBatteryPenalty() throws Exception {
    setField("minBatteryVoltage", 10.5);
    assertEquals(90, summary.getLastHealthScore());
  }

  @Test
  void testHighTempPenalty() throws Exception {
    setField("maxMotorTempCelsius", 70.0);
    assertEquals(95, summary.getLastHealthScore());
  }
}
