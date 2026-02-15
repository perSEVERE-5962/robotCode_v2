package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.MethodName.class)
class CANHealthTelemetryTest extends TelemetryTestBase {

  private ShooterTelemetry shooterTelemetry;
  private IndexerTelemetry indexerTelemetry;
  private IntakeTelemetry intakeTelemetry;
  private IntakeActuatorTelemetry intakeActuatorTelemetry;
  private HangerTelemetry hangerTelemetry;
  private VisionTelemetry visionTelemetry;
  private DriveTelemetry driveTelemetry;
  private CANHealthTelemetry telemetry;

  @BeforeEach
  void setUp() {
    shooterTelemetry = new ShooterTelemetry();
    indexerTelemetry = new IndexerTelemetry();
    intakeTelemetry = new IntakeTelemetry();
    intakeActuatorTelemetry = new IntakeActuatorTelemetry();
    hangerTelemetry = new HangerTelemetry();
    visionTelemetry = new VisionTelemetry();
    driveTelemetry = new DriveTelemetry();
    telemetry =
        new CANHealthTelemetry(
            shooterTelemetry,
            indexerTelemetry,
            intakeTelemetry,
            intakeActuatorTelemetry,
            hangerTelemetry,
            visionTelemetry,
            driveTelemetry);
  }

  @Test
  void testGetNameReturnsCANHealth() {
    assertEquals("CANHealth", telemetry.getName());
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
  void testDefaultsAllDisconnected() throws Exception {
    // No motor telemetry has been updated, so deviceConnected defaults to false
    telemetry.update();
    assertFalse(telemetry.isAllConnected());
    assertEquals(0, telemetry.getConnectedCount());
    String list = telemetry.getDisconnectedList();
    assertTrue(list.contains("Shooter"));
    assertTrue(list.contains("Indexer"));
    assertTrue(list.contains("Intake"));
    assertTrue(list.contains("IntakeActuator"));
    assertTrue(list.contains("Hanger"));
    assertTrue(list.contains("Gyro"));
    assertTrue(list.contains("LeftCam"));
    assertTrue(list.contains("RightCam"));
  }

  @Test
  void testAllConnected() throws Exception {
    // Set all devices as connected via reflection
    setField(shooterTelemetry, "deviceConnected", true);
    setField(indexerTelemetry, "deviceConnected", true);
    setField(intakeTelemetry, "deviceConnected", true);
    setField(intakeActuatorTelemetry, "deviceConnected", true);
    setField(hangerTelemetry, "deviceConnected", true);
    setField(driveTelemetry, "gyroConnected", true);
    setField(visionTelemetry, "leftCamConnected", true);
    setField(visionTelemetry, "rightCamConnected", true);

    telemetry.update();
    assertTrue(telemetry.isAllConnected());
    assertEquals(8, telemetry.getConnectedCount());
    assertEquals("", telemetry.getDisconnectedList());
  }

  @Test
  void testPartialDisconnect() throws Exception {
    // Connect all except Shooter and LeftCam
    setField(indexerTelemetry, "deviceConnected", true);
    setField(intakeTelemetry, "deviceConnected", true);
    setField(intakeActuatorTelemetry, "deviceConnected", true);
    setField(hangerTelemetry, "deviceConnected", true);
    setField(driveTelemetry, "gyroConnected", true);
    setField(visionTelemetry, "rightCamConnected", true);

    telemetry.update();
    assertFalse(telemetry.isAllConnected());
    assertEquals(6, telemetry.getConnectedCount());
    String list = telemetry.getDisconnectedList();
    assertTrue(list.contains("Shooter"));
    assertTrue(list.contains("LeftCam"));
    assertFalse(list.contains("Indexer"));
  }

  @Test
  void testFaultAggregation() throws Exception {
    setField(shooterTelemetry, "deviceFaultsRaw", 3);
    setField(indexerTelemetry, "deviceFaultsRaw", 5);

    telemetry.update();
    int totalFaults = getField(telemetry, "totalFaults");
    boolean hasFaults = telemetry.hasFaults();
    assertEquals(8, totalFaults);
    assertTrue(hasFaults);
  }

  @Test
  void testNoFaults() throws Exception {
    telemetry.update();
    int totalFaults = getField(telemetry, "totalFaults");
    assertFalse(telemetry.hasFaults());
    assertEquals(0, totalFaults);
  }

  @Test
  void testNullTelemetryHandling() {
    CANHealthTelemetry nullTelemetry =
        new CANHealthTelemetry(null, null, null, null, null, null, null);
    assertDoesNotThrow(
        () -> {
          nullTelemetry.update();
          nullTelemetry.log();
        });
    assertFalse(nullTelemetry.isAllConnected());
    assertEquals(0, nullTelemetry.getConnectedCount());
    // All 8 devices should be listed as disconnected
    String list = nullTelemetry.getDisconnectedList();
    assertTrue(list.contains("Shooter"));
    assertTrue(list.contains("RightCam"));
  }

  private void setField(Object obj, String fieldName, Object value) throws Exception {
    java.lang.reflect.Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    f.set(obj, value);
  }
}
