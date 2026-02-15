package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

/**
 * Full TelemetryManager lifecycle tests. Single instance reused across tests to avoid HAL resource
 * conflicts from subsystem singletons.
 */
@TestMethodOrder(MethodOrderer.MethodName.class)
class IntegrationTest {

  private static TelemetryManager manager;

  @BeforeAll
  static void initAll() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    manager = TelemetryManager.getInstance();
  }

  @AfterAll
  static void tearDownAll() {
    // Close SparkMax devices to free CAN IDs before nulling singletons
    closeSubsystemMotor("frc.robot.subsystems.Shooter");
    closeSubsystemMotor("frc.robot.subsystems.Indexer");
    closeSubsystemMotor("frc.robot.subsystems.Intake");
    resetSingleton("frc.robot.telemetry.TelemetryManager");
    resetSingleton("frc.robot.telemetry.CycleTracker");
    resetSingleton("frc.robot.subsystems.Shooter");
    resetSingleton("frc.robot.subsystems.Indexer");
    resetSingleton("frc.robot.subsystems.Intake");
  }

  @BeforeEach
  void setUp() {
    SafeLog.logAndReset();
  }

  @Test
  void testTelemetryManagerCreation() {
    assertNotNull(manager, "TelemetryManager singleton should not be null");
  }

  @Test
  void testUpdateAllDoesNotThrow() {
    assertDoesNotThrow(
        () -> manager.updateAll(), "updateAll() should not throw even without real hardware");
  }

  @Test
  void testMultipleUpdateCycles() {
    assertDoesNotThrow(
        () -> {
          for (int i = 0; i < 10; i++) {
            manager.updateAll();
          }
        },
        "10 consecutive updateAll() calls should not crash");
  }

  @Test
  void testAccessorsReturnDefaults() {
    manager.updateAll();

    assertTrue(manager.getLoopTimeMs() >= 0, "Loop time should be non-negative");
    assertFalse(manager.isShooterAtSpeed(), "Shooter should not be at speed without hardware");
    assertFalse(manager.isReadyToShoot(), "ReadyToShoot should be false without hardware");
    assertFalse(manager.isLockedOnTarget(), "Vision lock should be false without hardware");
    assertFalse(
        manager.isIndexerJamDetected(), "Indexer jam should not be detected without hardware");
    assertFalse(manager.isBrownoutRisk(), "Brownout risk should be false in test env");
    assertEquals(0, manager.getTotalShots(), "Shot count should be 0 without hardware");
  }

  @Test
  void testSafeLogCountersReset() {
    manager.updateAll();

    assertEquals(0, SafeLog.getCycleFailures(), "SafeLog cycle failures should be 0 after reset");
    assertEquals(
        "", SafeLog.getLastFailedKey(), "SafeLog last failed key should be empty after reset");
  }

  @Test
  void testCycleTrackerAccessibleFromManager() {
    manager.updateAll();

    CycleTracker tracker = CycleTracker.getInstance();
    assertNotNull(tracker);
    assertEquals(CycleTracker.CyclePhase.IDLE, tracker.getCurrentPhase());
  }

  @Test
  void testSecondUpdateAfterFirst() {
    manager.updateAll();

    assertDoesNotThrow(() -> manager.updateAll(), "Second updateAll() should also succeed");
  }

  @Test
  void testGetDriveTelemetryReturnsNonNull() {
    manager.updateAll();
    DriveTelemetry drive = manager.getDriveTelemetry();
    assertNotNull(drive, "DriveTelemetry accessor should return non-null");
  }

  private static void closeSubsystemMotor(String className) {
    try {
      Class<?> clazz = Class.forName(className);
      Field f = clazz.getDeclaredField("instance");
      f.setAccessible(true);
      Object instance = f.get(null);
      if (instance != null) {
        java.lang.reflect.Method getMotor = clazz.getMethod("getMotor");
        Object motor = getMotor.invoke(instance);
        if (motor instanceof AutoCloseable) {
          ((AutoCloseable) motor).close();
        }
      }
    } catch (Exception e) {
      // Class may not have been loaded or doesn't have getMotor
    }
  }

  private static void resetSingleton(String className) {
    try {
      Class<?> clazz = Class.forName(className);
      Field f = clazz.getDeclaredField("instance");
      f.setAccessible(true);
      f.set(null, null);
    } catch (Exception e) {
      // Class may not have been loaded yet
    }
  }
}
