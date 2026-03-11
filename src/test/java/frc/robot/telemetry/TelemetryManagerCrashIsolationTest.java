package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.lang.reflect.Field;
import java.util.List;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class TelemetryManagerCrashIsolationTest {

  private static TelemetryManager manager;

  @BeforeAll
  static void initAll() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    manager = TelemetryManager.getInstance();
  }

  @BeforeEach
  void setUp() {
    SafeLog.logAndReset();
  }

  @AfterAll
  static void tearDownAll() {
    closeSubsystemMotor("frc.robot.subsystems.Shooter");
    closeSubsystemMotor("frc.robot.subsystems.Indexer");
    closeSubsystemMotor("frc.robot.subsystems.Intake");
    resetSingleton("frc.robot.telemetry.TelemetryManager");

    resetSingleton("frc.robot.subsystems.Shooter");
    resetSingleton("frc.robot.subsystems.Indexer");
    resetSingleton("frc.robot.subsystems.Intake");
  }

  @Test
  void testAllNineteenTelemetryClassesRegistered() throws Exception {
    List<?> list = getField(manager, "telemetryList");
    assertEquals(20, list.size(), "TelemetryManager should register exactly 20 telemetry classes");
  }

  @Test
  void testThrowingClassDoesNotCrashUpdateAll() throws Exception {
    @SuppressWarnings("unchecked")
    List<SubsystemTelemetry> list = getField(manager, "telemetryList");
    SubsystemTelemetry thrower =
        new SubsystemTelemetry() {
          @Override
          public void update() {
            throw new RuntimeException("injected crash");
          }

          @Override
          public void log() {
            throw new RuntimeException("injected crash");
          }

          @Override
          public String getName() {
            return "TestCrasher";
          }
        };

    list.add(0, thrower);
    try {
      assertDoesNotThrow(() -> manager.updateAll(), "updateAll must survive a throwing class");
    } finally {
      list.remove(0);
    }
  }

  @Test
  void testFailureCountIncrements() throws Exception {
    @SuppressWarnings("unchecked")
    List<SubsystemTelemetry> list = getField(manager, "telemetryList");
    SubsystemTelemetry thrower =
        new SubsystemTelemetry() {
          @Override
          public void update() {
            throw new RuntimeException("injected crash");
          }

          @Override
          public void log() {
            throw new RuntimeException("injected crash");
          }

          @Override
          public String getName() {
            return "TestCrasher";
          }
        };

    list.add(0, thrower);
    try {
      manager.updateAll();
      int failures = getField(manager, "cycleFailures");
      assertTrue(failures > 0, "cycleFailures should increment when a class throws");
    } finally {
      list.remove(0);
    }
  }

  @Test
  void testLastFailedNameCaptured() throws Exception {
    @SuppressWarnings("unchecked")
    List<SubsystemTelemetry> list = getField(manager, "telemetryList");
    SubsystemTelemetry thrower =
        new SubsystemTelemetry() {
          @Override
          public void update() {
            throw new RuntimeException("injected crash");
          }

          @Override
          public void log() {}

          @Override
          public String getName() {
            return "CrashOnlyInUpdate";
          }
        };

    list.add(0, thrower);
    try {
      manager.updateAll();
      String lastFailed = getField(manager, "lastFailedName");
      assertTrue(
          lastFailed.contains("CrashOnlyInUpdate"),
          "lastFailedName should identify the crashing class");
    } finally {
      list.remove(0);
    }
  }

  @SuppressWarnings("unchecked")
  private <T> T getField(Object obj, String fieldName) throws Exception {
    Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    return (T) f.get(obj);
  }

  private static void resetSingleton(String className) {
    try {
      Class<?> clazz = Class.forName(className);
      Field f = clazz.getDeclaredField("instance");
      f.setAccessible(true);
      f.set(null, null);
    } catch (Exception e) {
    }
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
    }
  }
}
