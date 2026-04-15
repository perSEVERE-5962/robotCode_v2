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
  private AgitatorTelemetry agitatorTelemetry;
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
    agitatorTelemetry = new AgitatorTelemetry();
    visionTelemetry = new VisionTelemetry();
    driveTelemetry = new DriveTelemetry();
    telemetry =
        new CANHealthTelemetry(
            shooterTelemetry,
            indexerTelemetry,
            intakeTelemetry,
            intakeActuatorTelemetry,
            hangerTelemetry,
            agitatorTelemetry,
            visionTelemetry,
            driveTelemetry);
  }

  @Test
  void testDefaultsAllDisconnected() throws Exception {
    telemetry.update();
    assertFalse(telemetry.isAllConnected());
    assertEquals(0, telemetry.getConnectedCount());
    String list = telemetry.getDisconnectedList();
    assertTrue(list.contains("Shooter"));
    assertTrue(list.contains("Indexer"));
    assertTrue(list.contains("Intake"));
    assertTrue(list.contains("IntakeActuator"));
    assertTrue(list.contains("Hanger"));
    assertTrue(list.contains("Agitator"));
    assertTrue(list.contains("Gyro"));
    assertTrue(list.contains("LeftCam"));
    assertTrue(list.contains("RightCam"));
    assertTrue(list.contains("FL-Drive"));
    assertTrue(list.contains("BR-Encoder"));
  }

  @Test
  void testAllConnected() throws Exception {
    setField(shooterTelemetry, "deviceConnected", true);
    setField(indexerTelemetry, "deviceConnected", true);
    setField(intakeTelemetry, "deviceConnected", true);
    setField(intakeActuatorTelemetry, "deviceConnected", true);
    setField(hangerTelemetry, "deviceConnected", true);
    setField(agitatorTelemetry, "deviceConnected", true);
    setField(driveTelemetry, "gyroConnected", true);
    setField(visionTelemetry, "leftCamConnected", true);
    setField(visionTelemetry, "rightCamConnected", true);
    // Swerve module devices
    setField(driveTelemetry, "driveMotorConnected", new boolean[] {true, true, true, true});
    setField(driveTelemetry, "turnMotorConnected", new boolean[] {true, true, true, true});
    setField(driveTelemetry, "encoderReadIssue", new boolean[] {false, false, false, false});

    telemetry.update();
    assertTrue(telemetry.isAllConnected());
    assertEquals(21, telemetry.getConnectedCount());
    assertEquals("", telemetry.getDisconnectedList());
  }

  @Test
  void testPartialDisconnect() throws Exception {
    // Connect all mechanism devices
    setField(shooterTelemetry, "deviceConnected", true);
    setField(indexerTelemetry, "deviceConnected", true);
    setField(intakeTelemetry, "deviceConnected", true);
    setField(intakeActuatorTelemetry, "deviceConnected", true);
    setField(hangerTelemetry, "deviceConnected", true);
    setField(agitatorTelemetry, "deviceConnected", true);
    setField(driveTelemetry, "gyroConnected", true);
    setField(visionTelemetry, "leftCamConnected", true);
    setField(visionTelemetry, "rightCamConnected", true);
    // Connect most swerve, leave FR-Drive disconnected
    setField(driveTelemetry, "driveMotorConnected", new boolean[] {true, false, true, true});
    setField(driveTelemetry, "turnMotorConnected", new boolean[] {true, true, true, true});
    setField(driveTelemetry, "encoderReadIssue", new boolean[] {false, false, false, false});

    telemetry.update();
    assertFalse(telemetry.isAllConnected());
    assertEquals(20, telemetry.getConnectedCount());
    String list = telemetry.getDisconnectedList();
    assertTrue(list.contains("FR-Drive"), "Should name the specific disconnected module");
    assertFalse(list.contains("Shooter"));
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
  void testMissingCountAndCriticalWhenAllOnline() throws Exception {
    setField(shooterTelemetry, "deviceConnected", true);
    setField(indexerTelemetry, "deviceConnected", true);
    setField(intakeTelemetry, "deviceConnected", true);
    setField(intakeActuatorTelemetry, "deviceConnected", true);
    setField(hangerTelemetry, "deviceConnected", true);
    setField(agitatorTelemetry, "deviceConnected", true);
    setField(driveTelemetry, "gyroConnected", true);
    setField(visionTelemetry, "leftCamConnected", true);
    setField(visionTelemetry, "rightCamConnected", true);
    setField(driveTelemetry, "driveMotorConnected", new boolean[] {true, true, true, true});
    setField(driveTelemetry, "turnMotorConnected", new boolean[] {true, true, true, true});
    setField(driveTelemetry, "encoderReadIssue", new boolean[] {false, false, false, false});

    telemetry.update();
    assertEquals(0, telemetry.getMissingCount());
    assertFalse(telemetry.isMissingCritical());
    assertEquals(0, telemetry.getMissingNames().length);
    assertEquals(0, telemetry.getMissingIds().length);
  }

  @Test
  void testHangerMissingSetsCritical() throws Exception {
    setField(shooterTelemetry, "deviceConnected", true);
    setField(indexerTelemetry, "deviceConnected", true);
    setField(intakeTelemetry, "deviceConnected", true);
    setField(intakeActuatorTelemetry, "deviceConnected", true);
    setField(hangerTelemetry, "deviceConnected", false);
    setField(agitatorTelemetry, "deviceConnected", true);
    setField(driveTelemetry, "gyroConnected", true);
    setField(visionTelemetry, "leftCamConnected", true);
    setField(visionTelemetry, "rightCamConnected", true);
    setField(driveTelemetry, "driveMotorConnected", new boolean[] {true, true, true, true});
    setField(driveTelemetry, "turnMotorConnected", new boolean[] {true, true, true, true});
    setField(driveTelemetry, "encoderReadIssue", new boolean[] {false, false, false, false});

    telemetry.update();
    assertEquals(1, telemetry.getMissingCount());
    assertTrue(telemetry.isMissingCritical());
    String[] names = telemetry.getMissingNames();
    assertEquals(1, names.length);
    assertEquals("Hanger", names[0]);
  }

  @Test
  void testOnlyCameraMissingIsNotCritical() throws Exception {
    setField(shooterTelemetry, "deviceConnected", true);
    setField(indexerTelemetry, "deviceConnected", true);
    setField(intakeTelemetry, "deviceConnected", true);
    setField(intakeActuatorTelemetry, "deviceConnected", true);
    setField(hangerTelemetry, "deviceConnected", true);
    setField(agitatorTelemetry, "deviceConnected", true);
    setField(driveTelemetry, "gyroConnected", true);
    setField(visionTelemetry, "leftCamConnected", false);
    setField(visionTelemetry, "rightCamConnected", true);
    setField(driveTelemetry, "driveMotorConnected", new boolean[] {true, true, true, true});
    setField(driveTelemetry, "turnMotorConnected", new boolean[] {true, true, true, true});
    setField(driveTelemetry, "encoderReadIssue", new boolean[] {false, false, false, false});

    telemetry.update();
    assertEquals(1, telemetry.getMissingCount());
    assertFalse(telemetry.isMissingCritical(), "camera-only drop is not a critical flag");
    String[] names = telemetry.getMissingNames();
    assertEquals("LeftCam", names[0]);
  }

  private void setField(Object obj, String fieldName, Object value) throws Exception {
    java.lang.reflect.Field f = obj.getClass().getDeclaredField(fieldName);
    f.setAccessible(true);
    f.set(obj, value);
  }
}
