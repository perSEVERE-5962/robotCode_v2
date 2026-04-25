package frc.robot.telemetry;

import frc.robot.Constants;
import frc.robot.util.SafeLog;
import java.util.ArrayList;
import java.util.List;

/**
 * Aggregates device connectivity and fault status across all CAN devices. Reads from existing
 * telemetry classes only, no additional CAN reads.
 *
 * <p>Beyond the aggregate connected/disconnected rollup, this class owns the device inventory
 * contract the Pit Check page uses: an expected list of motor controllers, sensors, and cameras
 * with CAN IDs, subsystem names, and a device class. Missing devices are published as parallel
 * arrays so a dashboard can render "17 of 18 devices online, missing CAN 42 (IntakePivot)" without
 * any source side lookup.
 */
class CANHealthTelemetry implements SubsystemTelemetry {

  /** Coarse grouping for the dashboard. Motor controllers are "critical", cameras are not. */
  public enum DeviceClass {
    MOTOR_CONTROLLER,
    SENSOR,
    CAMERA,
    OTHER
  }

  /**
   * An expected device in the robot's hardware inventory. {@code canId} is -1 when the device is
   * not on CAN (gyro over MXP, cameras over NetworkTables).
   */
  private record ExpectedDevice(String name, int canId, DeviceClass deviceClass) {}

  private static final List<ExpectedDevice> EXPECTED =
      List.of(
          new ExpectedDevice(
              "Shooter", Constants.CANDeviceIDs.kShooterID, DeviceClass.MOTOR_CONTROLLER),
          new ExpectedDevice(
              "Indexer", Constants.CANDeviceIDs.kIndexerID, DeviceClass.MOTOR_CONTROLLER),
          new ExpectedDevice(
              "IntakeRoller", Constants.CANDeviceIDs.kIntakeRollerID, DeviceClass.MOTOR_CONTROLLER),
          new ExpectedDevice(
              "IntakePivot", Constants.CANDeviceIDs.kIntakePivotID, DeviceClass.MOTOR_CONTROLLER),
          new ExpectedDevice(
              "Hanger", Constants.CANDeviceIDs.kHangerID, DeviceClass.MOTOR_CONTROLLER),
          new ExpectedDevice(
              "Agitator", Constants.CANDeviceIDs.kAgitatorID, DeviceClass.MOTOR_CONTROLLER),
          new ExpectedDevice("Gyro", -1, DeviceClass.SENSOR),
          new ExpectedDevice("LeftCam", -1, DeviceClass.CAMERA),
          new ExpectedDevice("RightCam", -1, DeviceClass.CAMERA),
          new ExpectedDevice("FL-Drive", -1, DeviceClass.MOTOR_CONTROLLER),
          new ExpectedDevice("FL-Turn", -1, DeviceClass.MOTOR_CONTROLLER),
          new ExpectedDevice("FL-Encoder", -1, DeviceClass.SENSOR),
          new ExpectedDevice("FR-Drive", -1, DeviceClass.MOTOR_CONTROLLER),
          new ExpectedDevice("FR-Turn", -1, DeviceClass.MOTOR_CONTROLLER),
          new ExpectedDevice("FR-Encoder", -1, DeviceClass.SENSOR),
          new ExpectedDevice("BL-Drive", -1, DeviceClass.MOTOR_CONTROLLER),
          new ExpectedDevice("BL-Turn", -1, DeviceClass.MOTOR_CONTROLLER),
          new ExpectedDevice("BL-Encoder", -1, DeviceClass.SENSOR),
          new ExpectedDevice("BR-Drive", -1, DeviceClass.MOTOR_CONTROLLER),
          new ExpectedDevice("BR-Turn", -1, DeviceClass.MOTOR_CONTROLLER),
          new ExpectedDevice("BR-Encoder", -1, DeviceClass.SENSOR));

  private static final int TOTAL_DEVICES = EXPECTED.size();

  // Pre-sized. Lists reuse their backing array across cycles; the final
  // String[]/int[] at publish time are new each cycle because they become
  // the logged snapshot and must not share state with next cycle's scratch.
  private final boolean[] connected = new boolean[TOTAL_DEVICES];
  private final List<String> missingNamesScratch = new ArrayList<>(TOTAL_DEVICES);
  private final List<Integer> missingIdsScratch = new ArrayList<>(TOTAL_DEVICES);

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
  private int missingCount = 0;
  private boolean missingCritical = false;
  private String[] missingNames = new String[0];
  private int[] missingIds = new int[0];
  private String missingNamesJoined = "";
  private String statusMessage = "READY";
  private String disconnectedList = "";

  private int totalFaults = 0;
  private boolean hasFaults = false;

  private int txErrorPeak = 0;
  private int rxErrorPeak = 0;
  private int txFullEvents = 0;
  private int errorPassiveEvents = 0;
  private int lastTxFullCount = 0;
  private boolean wasErrorPassive = false;

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

    missingNamesScratch.clear();
    missingIdsScratch.clear();
    int count = 0;
    StringBuilder disconnected = new StringBuilder();
    boolean critical = false;

    for (int i = 0; i < TOTAL_DEVICES; i++) {
      ExpectedDevice dev = EXPECTED.get(i);
      if (connected[i]) {
        count++;
      } else {
        missingNamesScratch.add(dev.name());
        missingIdsScratch.add(dev.canId());
        if (dev.deviceClass() == DeviceClass.MOTOR_CONTROLLER) {
          critical = true;
        }
        if (disconnected.length() > 0) {
          disconnected.append(", ");
        }
        disconnected.append(dev.name());
      }
    }

    connectedCount = count;
    missingCount = TOTAL_DEVICES - count;
    missingCritical = critical;
    allConnected = (count == TOTAL_DEVICES);
    disconnectedList = disconnected.toString();
    missingNamesJoined = disconnectedList.isEmpty() ? "none" : disconnectedList;
    missingNames = missingNamesScratch.toArray(new String[0]);
    missingIds = new int[missingIdsScratch.size()];
    for (int i = 0; i < missingIdsScratch.size(); i++) {
      missingIds[i] = missingIdsScratch.get(i);
    }

    // Dashboard-friendly short status line that fits inside a Text Display tile.
    if (allConnected) {
      statusMessage = "READY - all 21 devices online";
    } else if (critical) {
      statusMessage = "DO NOT QUEUE - " + missingCount + " missing: " + missingNamesJoined;
    } else {
      statusMessage = "WARNING - " + missingCount + " non-critical missing: " + missingNamesJoined;
    }

    // Sum fault bits. -1 means failed read, skip to avoid masking real faults.
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

    // CAN error peak tracking: read whatever SystemHealthTelemetry already
    // captured. Defensive: if the shared values have not been seen yet, skip.
    int currentTx = SystemHealthTelemetry.getCanTxErrorsSafely();
    int currentRx = SystemHealthTelemetry.getCanRxErrorsSafely();
    int currentTxFull = SystemHealthTelemetry.getCanTxFullSafely();
    if (currentTx >= 0) {
      if (currentTx > txErrorPeak) txErrorPeak = currentTx;
      // Entered error-passive when TxError crosses 127. Edge-triggered counter.
      boolean nowPassive = currentTx > 127;
      if (nowPassive && !wasErrorPassive) {
        errorPassiveEvents++;
      }
      wasErrorPassive = nowPassive;
    }
    if (currentRx >= 0 && currentRx > rxErrorPeak) {
      rxErrorPeak = currentRx;
    }
    if (currentTxFull >= 0) {
      if (currentTxFull > lastTxFullCount) {
        txFullEvents++;
      }
      lastTxFullCount = currentTxFull;
    }
  }

  @Override
  public void log() {
    SafeLog.put("CANHealth/AllConnected", allConnected);
    SafeLog.put("CANHealth/ExpectedCount", TOTAL_DEVICES);
    SafeLog.put("CANHealth/ConnectedCount", connectedCount);
    SafeLog.put("CANHealth/MissingCount", missingCount);
    SafeLog.put("CANHealth/MissingCritical", missingCritical);
    SafeLog.put("CANHealth/MissingNames", missingNames);
    SafeLog.put("CANHealth/MissingIds", missingIds);
    SafeLog.put("CANHealth/MissingNamesJoined", missingNamesJoined);
    SafeLog.put("CANHealth/StatusMessage", statusMessage);
    SafeLog.put("CANHealth/TotalFaults", totalFaults);
    SafeLog.put("CANHealth/HasFaults", hasFaults);
    SafeLog.put("CANHealth/TxErrorPeak", txErrorPeak);
    SafeLog.put("CANHealth/RxErrorPeak", rxErrorPeak);
    SafeLog.put("CANHealth/TxFullEvents", txFullEvents);
    SafeLog.put("CANHealth/ErrorPassiveEvents", errorPassiveEvents);

    // Legacy signals kept for backward compatibility with existing dashboards.
    SafeLog.put("SystemHealth/CAN/AllDevicesConnected", allConnected);
    SafeLog.put("SystemHealth/CAN/ConnectedCount", connectedCount);
    SafeLog.put("SystemHealth/CAN/TotalDevices", TOTAL_DEVICES);
    SafeLog.put("SystemHealth/CAN/TotalFaults", totalFaults);
    SafeLog.put("SystemHealth/CAN/HasFaults", hasFaults);
    if (!allConnected) {
      SafeLog.put("SystemHealth/CAN/DisconnectedList", disconnectedList);
    }
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

  public int getMissingCount() {
    return missingCount;
  }

  public boolean isMissingCritical() {
    return missingCritical;
  }

  public String[] getMissingNames() {
    return missingNames;
  }

  public int[] getMissingIds() {
    return missingIds;
  }

  public String getDisconnectedList() {
    return disconnectedList;
  }

  public boolean hasFaults() {
    return hasFaults;
  }

  public int getTxErrorPeak() {
    return txErrorPeak;
  }

  public int getRxErrorPeak() {
    return rxErrorPeak;
  }

  public int getTxFullEvents() {
    return txFullEvents;
  }

  public int getErrorPassiveEvents() {
    return errorPassiveEvents;
  }
}
