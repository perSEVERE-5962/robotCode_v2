package frc.robot.telemetry;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

public abstract class TelemetryTestBase {
  @BeforeEach
  void baseSetUp() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void baseTearDown() {
    // Close SparkMax devices to free CAN IDs before nulling singletons
    closeSubsystemMotor("frc.robot.subsystems.Shooter");
    closeSubsystemMotor("frc.robot.subsystems.Indexer");
    closeSubsystemMotor("frc.robot.subsystems.Intake");
    resetSingleton("frc.robot.telemetry.TelemetryManager");
    resetSingleton("frc.robot.telemetry.CycleTracker");
    resetSingleton("frc.robot.subsystems.Shooter");
    resetSingleton("frc.robot.subsystems.Indexer");
    resetSingleton("frc.robot.subsystems.Intake");
    SafeLog.logAndReset();
  }

  protected static void resetSingleton(String className) {
    try {
      Class<?> clazz = Class.forName(className);
      Field f = clazz.getDeclaredField("instance");
      f.setAccessible(true);
      f.set(null, null);
    } catch (Exception e) {
      // Class may not have been loaded yet
    }
  }

  /** Close SparkMax motor to free CAN device ID before singleton reset */
  protected static void closeSubsystemMotor(String className) {
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

  @SuppressWarnings("unchecked")
  protected <T> T getField(Object obj, String fieldName) throws Exception {
    Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    return (T) f.get(obj);
  }

  /** Sleep + pump DriverStation data to advance sim time */
  protected void advanceTime() {
    try {
      Thread.sleep(25);
    } catch (InterruptedException e) {
      /* ignored */
    }
    DriverStationSim.notifyNewData();
  }
}
