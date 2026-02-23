package frc.robot.telemetry;

import com.revrobotics.spark.SparkSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;

/**
 * Shared base for SparkSim-backed telemetry tests (Shooter, Indexer, Intake). Uses @BeforeAll to
 * create subsystem singletons once per test class.
 */
public abstract class SparkSimTestBase {

  protected static SparkSim shooterSim;
  protected static SparkSim indexerSim;
  protected static SparkSim intakeSim;

  @BeforeAll
  static void initHardware() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();

    // Nominal battery voltage
    RoboRioSim.setVInVoltage(12.5);

    // Create singletons (each creates a SparkMax on the CAN bus)
    Shooter shooter = Shooter.getInstance();
    Indexer indexer = Indexer.getInstance();
    Intake intake = Intake.getInstance();

    // Wrap with SparkSim for physics feedback
    shooterSim = new SparkSim(shooter.getMotor(), DCMotor.getNEO(1));
    indexerSim = new SparkSim(indexer.getMotor(), DCMotor.getNEO(1));
    intakeSim = new SparkSim(intake.getMotor(), DCMotor.getNEO(1));
  }

  @BeforeEach
  void resetState() {
    SafeLog.logAndReset();
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    RoboRioSim.setVInVoltage(12.5);
  }

  @AfterAll
  static void tearDownHardware() {
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

  protected static void closeSubsystemMotor(String className) {
    try {
      Class<?> clazz = Class.forName(className);
      java.lang.reflect.Field f = clazz.getDeclaredField("instance");
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

  protected static void resetSingleton(String className) {
    try {
      Class<?> clazz = Class.forName(className);
      java.lang.reflect.Field f = clazz.getDeclaredField("instance");
      f.setAccessible(true);
      f.set(null, null);
    } catch (Exception e) {
      // Class may not have been loaded yet
    }
  }

  /** Iterate SparkSim physics for one cycle at given velocity */
  protected static void iterateMotor(SparkSim sim, double velocityRPM, double busVoltage) {
    sim.iterate(velocityRPM, busVoltage, 0.020);
  }

  /** Set encoder velocity directly, bypasses firmware filtering for deterministic tests */
  protected static void setMotorVelocity(SparkSim sim, double velocityRPM) {
    sim.getRelativeEncoderSim().setVelocity(velocityRPM);
  }

  /** Pump telemetry update() N times with DS notify between each */
  protected void runCycles(SubsystemTelemetry telemetry, int cycles) {
    for (int i = 0; i < cycles; i++) {
      telemetry.update();
      DriverStationSim.notifyNewData();
    }
  }

  /** Access private field via reflection */
  @SuppressWarnings("unchecked")
  protected <T> T getField(Object obj, String fieldName) throws Exception {
    java.lang.reflect.Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    return (T) f.get(obj);
  }
}
