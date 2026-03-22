package frc.robot.telemetry;

/**
 * Aggregates device connectivity and fault status across all CAN devices. Reads from existing
 * telemetry classes only, no additional CAN reads.
 */
public class CANHealthTelemetry implements SubsystemTelemetry {
  private static final int TOTAL_DEVICES = 21; // 6 motors + gyro + 2 cameras + 12 swerve
  private static final String[] DEVICE_NAMES = {
    "Shooter",
    "Indexer",
    "IntakeRoller",
    "IntakePivot",
    "Hanger",
    "Agitator",
    "Gyro",
    "LeftCam",
    "RightCam",
    "FL-Drive",
    "FL-Turn",
    "FL-Encoder",
    "FR-Drive",
    "FR-Turn",
    "FR-Encoder",
    "BL-Drive",
    "BL-Turn",
    "BL-Encoder",
    "BR-Drive",
    "BR-Turn",
    "BR-Encoder"
  };

  private final ShooterTelemetry shooterTelemetry;
  private final IndexerTelemetry indexerTelemetry;
  private final IntakeRollerTelemetry intakeRollerTelemetry;
  private final IntakePivotTelemetry intakePivotTelemetry;
  private final HangerTelemetry hangerTelemetry;
  private final AgitatorTelemetry agitatorTelemetry;
  private final VisionTelemetry visionTelemetry;
  private final DriveTelemetry driveTelemetry;

  private boolean allConnected = false;
  private int connectedCount = 0;
  private String disconnectedList = "";
  private int totalFaults = 0;
  private boolean hasFaults = false;

  public CANHealthTelemetry(
      ShooterTelemetry shooterTelemetry,
      IndexerTelemetry indexerTelemetry,
      IntakeRollerTelemetry intakeRollerTelemetry,
      IntakePivotTelemetry intakePivotTelemetry,
      HangerTelemetry hangerTelemetry,
      AgitatorTelemetry agitatorTelemetry,
      VisionTelemetry visionTelemetry,
      DriveTelemetry driveTelemetry) {
    this.shooterTelemetry = shooterTelemetry;
    this.indexerTelemetry = indexerTelemetry;
    this.intakeRollerTelemetry = intakeRollerTelemetry;
    this.intakePivotTelemetry = intakePivotTelemetry;
    this.hangerTelemetry = hangerTelemetry;
    this.agitatorTelemetry = agitatorTelemetry;
    this.visionTelemetry = visionTelemetry;
    this.driveTelemetry = driveTelemetry;
  }

  @Override
  public void update() {
    boolean[] connected = new boolean[TOTAL_DEVICES];

    connected[0] = shooterTelemetry != null && shooterTelemetry.isDeviceConnected();
    connected[1] = indexerTelemetry != null && indexerTelemetry.isDeviceConnected();
    connected[2] = intakeRollerTelemetry != null && intakeRollerTelemetry.isDeviceConnected();
    connected[3] = intakePivotTelemetry != null && intakePivotTelemetry.isDeviceConnected();
    connected[4] = hangerTelemetry != null && hangerTelemetry.isDeviceConnected();
    connected[5] = agitatorTelemetry != null && agitatorTelemetry.isDeviceConnected();
    connected[6] = driveTelemetry != null && driveTelemetry.isGyroConnected();
    connected[7] = visionTelemetry != null && visionTelemetry.isLeftCamConnected();
    connected[8] = visionTelemetry != null && visionTelemetry.isRightCamConnected();

    for (int m = 0; m < 4; m++) {
      int base = 9 + m * 3;
      connected[base] = driveTelemetry != null && driveTelemetry.isDriveMotorConnected(m);
      connected[base + 1] = driveTelemetry != null && driveTelemetry.isTurnMotorConnected(m);
      connected[base + 2] = driveTelemetry != null && driveTelemetry.isEncoderOk(m);
    }

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

    // -1 means failed read, skip to avoid masking real faults
    int faults = 0;
    if (shooterTelemetry != null) faults += Math.max(0, shooterTelemetry.getDeviceFaultsRaw());
    if (indexerTelemetry != null) faults += Math.max(0, indexerTelemetry.getDeviceFaultsRaw());
    if (intakeRollerTelemetry != null)
      faults += Math.max(0, intakeRollerTelemetry.getDeviceFaultsRaw());
    if (intakePivotTelemetry != null)
      faults += Math.max(0, intakePivotTelemetry.getDeviceFaultsRaw());
    if (hangerTelemetry != null) faults += Math.max(0, hangerTelemetry.getDeviceFaultsRaw());
    if (agitatorTelemetry != null) faults += Math.max(0, agitatorTelemetry.getDeviceFaultsRaw());
    if (driveTelemetry != null) {
      for (int m = 0; m < 4; m++) {
        faults += Math.max(0, driveTelemetry.getDriveFaultsRaw(m));
        faults += Math.max(0, driveTelemetry.getTurnFaultsRaw(m));
      }
    }
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
