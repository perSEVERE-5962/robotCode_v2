package frc.robot.telemetry;

/**
 * Aggregates device connectivity and fault status across all CAN devices. Reads from existing
 * telemetry classes only, no additional CAN reads.
 */
public class CANHealthTelemetry implements SubsystemTelemetry {
  private static final int TOTAL_DEVICES = 8; // 5 motors + gyro + 2 cameras
  private static final String[] DEVICE_NAMES = {
    "Shooter", "Indexer", "Intake", "IntakeActuator", "Hanger", "Gyro", "LeftCam", "RightCam"
  };

  private final ShooterTelemetry shooterTelemetry;
  private final IndexerTelemetry indexerTelemetry;
  private final IntakeTelemetry intakeTelemetry;
  private final IntakeActuatorTelemetry intakeActuatorTelemetry;
  private final HangerTelemetry hangerTelemetry;
  private final VisionTelemetry visionTelemetry;
  private final DriveTelemetry driveTelemetry;

  // State
  private boolean allConnected = false;
  private int connectedCount = 0;
  private String disconnectedList = "";
  private int totalFaults = 0;
  private boolean hasFaults = false;

  public CANHealthTelemetry(
      ShooterTelemetry shooterTelemetry,
      IndexerTelemetry indexerTelemetry,
      IntakeTelemetry intakeTelemetry,
      IntakeActuatorTelemetry intakeActuatorTelemetry,
      HangerTelemetry hangerTelemetry,
      VisionTelemetry visionTelemetry,
      DriveTelemetry driveTelemetry) {
    this.shooterTelemetry = shooterTelemetry;
    this.indexerTelemetry = indexerTelemetry;
    this.intakeTelemetry = intakeTelemetry;
    this.intakeActuatorTelemetry = intakeActuatorTelemetry;
    this.hangerTelemetry = hangerTelemetry;
    this.visionTelemetry = visionTelemetry;
    this.driveTelemetry = driveTelemetry;
  }

  @Override
  public void update() {
    boolean[] connected = new boolean[TOTAL_DEVICES];

    connected[0] = shooterTelemetry != null && shooterTelemetry.isDeviceConnected();
    connected[1] = indexerTelemetry != null && indexerTelemetry.isDeviceConnected();
    connected[2] = intakeTelemetry != null && intakeTelemetry.isDeviceConnected();
    connected[3] = intakeActuatorTelemetry != null && intakeActuatorTelemetry.isDeviceConnected();
    connected[4] = hangerTelemetry != null && hangerTelemetry.isDeviceConnected();
    connected[5] = driveTelemetry != null && driveTelemetry.isGyroConnected();
    connected[6] = visionTelemetry != null && visionTelemetry.isLeftCamConnected();
    connected[7] = visionTelemetry != null && visionTelemetry.isRightCamConnected();

    int count = 0;
    StringBuilder disconnected = new StringBuilder();
    for (int i = 0; i < TOTAL_DEVICES; i++) {
      if (connected[i]) {
        count++;
      } else {
        if (disconnected.length() > 0) {
          disconnected.append(", ");
        }
        disconnected.append(DEVICE_NAMES[i]);
      }
    }

    connectedCount = count;
    allConnected = (count == TOTAL_DEVICES);
    disconnectedList = disconnected.toString();

    // Fault aggregation across 5 motor telemetry classes
    // -1 sentinel means CAN read failed; clamp to 0 so failures don't hide real faults
    int faults = 0;
    if (shooterTelemetry != null) faults += Math.max(0, shooterTelemetry.getDeviceFaultsRaw());
    if (indexerTelemetry != null) faults += Math.max(0, indexerTelemetry.getDeviceFaultsRaw());
    if (intakeTelemetry != null) faults += Math.max(0, intakeTelemetry.getDeviceFaultsRaw());
    if (intakeActuatorTelemetry != null)
      faults += Math.max(0, intakeActuatorTelemetry.getDeviceFaultsRaw());
    if (hangerTelemetry != null) faults += Math.max(0, hangerTelemetry.getDeviceFaultsRaw());
    totalFaults = faults;
    hasFaults = (faults > 0);
  }

  @Override
  public void log() {
    SafeLog.put("SystemHealth/CAN/AllDevicesConnected", allConnected);
    SafeLog.put("SystemHealth/CAN/ConnectedCount", connectedCount);
    SafeLog.put("SystemHealth/CAN/TotalDevices", TOTAL_DEVICES);
    SafeLog.put("SystemHealth/CAN/DisconnectedList", disconnectedList);
    SafeLog.put("SystemHealth/CAN/TotalFaults", totalFaults);
    SafeLog.put("SystemHealth/CAN/HasFaults", hasFaults);
  }

  @Override
  public String getName() {
    return "CANHealth";
  }

  // Accessors for TelemetryManager
  public boolean isAllConnected() {
    return allConnected;
  }

  public int getConnectedCount() {
    return connectedCount;
  }

  public String getDisconnectedList() {
    return disconnectedList;
  }

  public boolean hasFaults() {
    return hasFaults;
  }
}
