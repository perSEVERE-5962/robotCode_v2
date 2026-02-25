package frc.robot.sim;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.telemetry.SafeLog;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Hub shift practice scenario for driver training. Runs a full teleop period (140s) with real-time
 * match clock countdown so the driver can feel all hub shift haptics:
 *
 * <ul>
 *   <li>HUB_ACTIVATED: hub becomes active (right-only pulse)
 *   <li>HUB_DEACTIVATED: hub becomes inactive (left-only buzz)
 *   <li>HUB_SHIFT_WARNING: 2.5s before each shift boundary (triple pulse)
 * </ul>
 *
 * <p>Hub shift schedule (REBUILT Section 6.4):
 *
 * <pre>
 *   Transition (140-130): BOTH ACTIVE
 *   Shift 1 (130-105): Winner INACTIVE, Loser ACTIVE
 *   Shift 2 (105-80):  Winner ACTIVE, Loser INACTIVE
 *   Shift 3 (80-55):   Winner INACTIVE, Loser ACTIVE
 *   Shift 4 (55-30):   Winner ACTIVE, Loser INACTIVE
 *   Endgame (30-0):    BOTH ACTIVE
 * </pre>
 *
 * <p>Dashboard controls (SmartDashboard):
 *
 * <ul>
 *   <li>HubPractice/WonAuto (boolean): toggle alliance parity (default true = Red won auto)
 *   <li>HubPractice/TimeScale (double): countdown speed multiplier (default 1.0, set 3.0 for 3x
 *       fast)
 *   <li>HubPractice/Restart (boolean): toggle to restart the countdown from 140s
 * </ul>
 *
 * <p>Logged signals for dashboard:
 *
 * <ul>
 *   <li>HubPractice/SimMatchTime: current simulated match time
 *   <li>HubPractice/ShiftLabel: human-readable shift name
 *   <li>HubPractice/Countdown: seconds until next shift boundary
 * </ul>
 *
 * <p>Launch: ./gradlew simulateJava -DsimScenario=HubShiftPractice
 */
public class HubShiftPracticeScenario implements SimScenario {
  private double startTime;
  private double lastUpdateTime;
  private double simMatchTime; // counts down from 140
  private boolean finished = false;
  private String lastShiftLabel = "";

  // Hub shift boundaries (match time, descending)
  private static final double TELEOP_START = 140.0;
  private static final double SHIFT_1_START = 130.0;
  private static final double SHIFT_2_START = 105.0;
  private static final double SHIFT_3_START = 80.0;
  private static final double SHIFT_4_START = 55.0;
  private static final double ENDGAME_START = 30.0;

  // Setup delay before teleop enable
  private static final double SETUP_DELAY = 2.0;

  @Override
  public String getName() {
    return "HubShiftPractice";
  }

  @Override
  public void init() {
    startTime = Timer.getFPGATimestamp();
    lastUpdateTime = startTime;
    simMatchTime = TELEOP_START;
    finished = false;
    lastShiftLabel = "";

    SimulatedBattery.disableBatterySim();
    RoboRioSim.setVInVoltage(12.8);

    // Dashboard controls with defaults
    SmartDashboard.putBoolean("HubPractice/WonAuto", true);
    SmartDashboard.putNumber("HubPractice/TimeScale", 1.0);
    SmartDashboard.putBoolean("HubPractice/Restart", false);

    // Set alliance (Red1, wonAuto=true means Red won auto)
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);

    // Start disabled briefly, then enable teleop
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();

    SafeLog.put("HubPractice/SimMatchTime", simMatchTime);
    SafeLog.put("HubPractice/ShiftLabel", "SETUP");
    SafeLog.put("HubPractice/Countdown", 0.0);
  }

  @Override
  public void execute(double timestampSec) {
    double wallElapsed = timestampSec - startTime;

    // Setup phase: wait briefly, then enable teleop
    if (wallElapsed < SETUP_DELAY) {
      SafeLog.put("HubPractice/ShiftLabel", "SETUP");
      return;
    }

    // Enable teleop on first frame after setup
    if (!DriverStationSim.getEnabled()) {
      DriverStationSim.setEnabled(true);
      DriverStationSim.setAutonomous(false);
      DriverStationSim.notifyNewData();
      lastUpdateTime = timestampSec;
    }

    // Check for restart toggle
    boolean restart = SmartDashboard.getBoolean("HubPractice/Restart", false);
    if (restart) {
      SmartDashboard.putBoolean("HubPractice/Restart", false);
      simMatchTime = TELEOP_START;
      lastUpdateTime = timestampSec;
    }

    // Read dashboard controls
    double timeScale = SmartDashboard.getNumber("HubPractice/TimeScale", 1.0);
    timeScale = Math.max(0.25, Math.min(10.0, timeScale)); // clamp to [0.25, 10]
    boolean wonAuto = SmartDashboard.getBoolean("HubPractice/WonAuto", true);

    // Advance match time countdown
    double dt = timestampSec - lastUpdateTime;
    lastUpdateTime = timestampSec;
    simMatchTime -= dt * timeScale;

    // Loop: when match time hits 0, restart from 140
    if (simMatchTime <= 0) {
      simMatchTime = TELEOP_START;
    }

    // Push simulated match time to DriverStation
    DriverStationSim.setMatchTime(simMatchTime);
    SmartDashboard.putBoolean("WonAuto", wonAuto);
    DriverStationSim.notifyNewData();

    // Compute shift label and countdown for dashboard
    String shiftLabel = computeShiftLabel(simMatchTime, wonAuto);
    double countdown = computeCountdown(simMatchTime);

    // Log for dashboard display
    SafeLog.put("HubPractice/SimMatchTime", simMatchTime);
    SafeLog.put("HubPractice/ShiftLabel", shiftLabel);
    SafeLog.put("HubPractice/Countdown", countdown);

    // Log phase transitions
    if (!shiftLabel.equals(lastShiftLabel)) {
      SafeLog.put("HubPractice/PreviousShift", lastShiftLabel);
      lastShiftLabel = shiftLabel;
    }
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  private String computeShiftLabel(double matchTime, boolean wonAuto) {
    if (matchTime > SHIFT_1_START) {
      return "TRANSITION (BOTH ACTIVE)";
    }
    int shift = computeShiftNumber(matchTime);
    boolean hubActive = computeHubActive(matchTime, wonAuto);
    if (matchTime <= ENDGAME_START) {
      return "ENDGAME (BOTH ACTIVE)";
    }
    return "SHIFT " + shift + (hubActive ? " (YOUR HUB ACTIVE)" : " (YOUR HUB INACTIVE)");
  }

  private double computeCountdown(double matchTime) {
    if (matchTime > SHIFT_1_START) return matchTime - SHIFT_1_START;
    if (matchTime > SHIFT_2_START) return matchTime - SHIFT_2_START;
    if (matchTime > SHIFT_3_START) return matchTime - SHIFT_3_START;
    if (matchTime > SHIFT_4_START) return matchTime - SHIFT_4_START;
    if (matchTime > ENDGAME_START) return matchTime - ENDGAME_START;
    return matchTime; // countdown to end of match
  }

  private int computeShiftNumber(double matchTime) {
    if (matchTime > SHIFT_1_START) return 0;
    if (matchTime > SHIFT_2_START) return 1;
    if (matchTime > SHIFT_3_START) return 2;
    if (matchTime > SHIFT_4_START) return 3;
    if (matchTime > ENDGAME_START) return 4;
    return 5;
  }

  private boolean computeHubActive(double matchTime, boolean wonAuto) {
    if (matchTime > SHIFT_1_START || matchTime <= ENDGAME_START) {
      return true; // transition or endgame
    }
    int shift = computeShiftNumber(matchTime);
    boolean oddShift = (shift % 2 == 1);
    return wonAuto ? !oddShift : oddShift;
  }
}
