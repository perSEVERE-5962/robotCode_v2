package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShotPredictorTelemetryTest extends TelemetryTestBase {
  private ShotPredictorTelemetry telemetry;

  @BeforeEach
  void setUp() throws Exception {
    // Reset ShootOnTheMove static fields via reflection
    Class<?> clazz = Class.forName("frc.robot.commands.ShootOnTheMove");
    setStaticField(clazz, "active", false);
    setStaticField(clazz, "snapDistanceToHubM", 0.0);
    setStaticField(clazz, "snapTimeOfFlightSec", 0.0);
    setStaticField(clazz, "snapCompTargetX", 0.0);
    setStaticField(clazz, "snapCompTargetY", 0.0);
    setStaticField(clazz, "snapDriftX", 0.0);
    setStaticField(clazz, "snapDriftY", 0.0);
    setStaticField(clazz, "snapComputedRPM", 0.0);
    setStaticField(clazz, "snapHeadingErrorRad", 0.0);
    setStaticField(clazz, "snapHeadingSpeedRadPerSec", 0.0);

    telemetry = new ShotPredictorTelemetry();
  }

  private void setStaticField(Class<?> clazz, String name, Object value) throws Exception {
    Field f = clazz.getDeclaredField(name);
    f.setAccessible(true);
    f.set(null, value);
  }

  @Test
  void testUpdateDoesNotThrow() {
    assertDoesNotThrow(
        () -> {
          telemetry.update();
          telemetry.log();
        });
  }

  @Test
  void testGetNameReturnsShotPredictor() {
    assertEquals("ShotPredictor", telemetry.getName());
  }

  @Test
  void testInactiveByDefault() {
    telemetry.update();
    assertFalse(telemetry.isCommandActive());
  }

  @Test
  void testDefaultsZeroWhenInactive() {
    telemetry.update();
    assertEquals(0, telemetry.getComputedRPM());
    assertEquals(0, telemetry.getHeadingErrorRad());
  }

  @Test
  void testReadsSnapshotWhenActive() throws Exception {
    Class<?> clazz = Class.forName("frc.robot.commands.ShootOnTheMove");
    setStaticField(clazz, "active", true);
    setStaticField(clazz, "snapDistanceToHubM", 4.5);
    setStaticField(clazz, "snapComputedRPM", 3800.0);
    setStaticField(clazz, "snapHeadingErrorRad", 0.12);

    telemetry.update();

    assertTrue(telemetry.isCommandActive());
    assertEquals(3800.0, telemetry.getComputedRPM(), 0.01);
    assertEquals(0.12, telemetry.getHeadingErrorRad(), 0.001);
  }

  @Test
  void testClearsWhenDeactivated() throws Exception {
    Class<?> clazz = Class.forName("frc.robot.commands.ShootOnTheMove");
    setStaticField(clazz, "active", true);
    setStaticField(clazz, "snapComputedRPM", 3800.0);
    telemetry.update();
    assertEquals(3800.0, telemetry.getComputedRPM(), 0.01);

    // Deactivate
    setStaticField(clazz, "active", false);
    telemetry.update();
    assertFalse(telemetry.isCommandActive());
    assertEquals(0, telemetry.getComputedRPM());
  }

  @Test
  void testLogDoesNotThrow() {
    telemetry.update();
    assertDoesNotThrow(() -> telemetry.log());
  }

  @Test
  void testLogDoesNotThrowWhenActive() throws Exception {
    Class<?> clazz = Class.forName("frc.robot.commands.ShootOnTheMove");
    setStaticField(clazz, "active", true);
    setStaticField(clazz, "snapDistanceToHubM", 3.0);
    setStaticField(clazz, "snapTimeOfFlightSec", 0.6);
    setStaticField(clazz, "snapCompTargetX", 11.5);
    setStaticField(clazz, "snapCompTargetY", 3.8);
    setStaticField(clazz, "snapDriftX", 0.4);
    setStaticField(clazz, "snapDriftY", -0.2);
    setStaticField(clazz, "snapComputedRPM", 4100.0);
    setStaticField(clazz, "snapHeadingErrorRad", -0.05);
    setStaticField(clazz, "snapHeadingSpeedRadPerSec", -0.15);

    telemetry.update();
    assertDoesNotThrow(() -> telemetry.log());
  }
}
