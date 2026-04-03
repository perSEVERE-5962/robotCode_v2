package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.HubTimingConstants;
import frc.robot.telemetry.SafeLog;

/**
 * Tracks which hub is active based on the shift schedule, auto winner, and TOF-shifted boundaries.
 * Call initializeTeleop() at teleop start, update() every robotPeriodic().
 */
public class HubShiftEngine {
  private static HubShiftEngine instance;

  public enum ShiftPhase {
    DISABLED,
    AUTO,
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME
  }

  public enum ScheduleConfidence {
    HIGH,
    LOW,
    FALLBACK
  }

  public record ShiftInfo(
      ShiftPhase phase,
      boolean hubActive,
      double elapsedInPhase,
      double remainingInState,
      double timeToNextActive,
      double timeUntilDeactivation) {}

  private static final double[] PHASE_STARTS = {
    0.0,
    HubTimingConstants.TRANSITION_END,
    HubTimingConstants.SHIFT1_END,
    HubTimingConstants.SHIFT2_END,
    HubTimingConstants.SHIFT3_END,
    HubTimingConstants.SHIFT4_END
  };
  private static final double[] PHASE_ENDS = {
    HubTimingConstants.TRANSITION_END,
    HubTimingConstants.SHIFT1_END,
    HubTimingConstants.SHIFT2_END,
    HubTimingConstants.SHIFT3_END,
    HubTimingConstants.SHIFT4_END,
    HubTimingConstants.ENDGAME_END
  };
  private static final ShiftPhase[] PHASES = {
    ShiftPhase.TRANSITION,
    ShiftPhase.SHIFT1,
    ShiftPhase.SHIFT2,
    ShiftPhase.SHIFT3,
    ShiftPhase.SHIFT4,
    ShiftPhase.ENDGAME
  };

  // won auto = our hub was active first, so inactive in shift1
  private static final boolean[] WON_SCHEDULE = {true, false, true, false, true, true};
  private static final boolean[] LOST_SCHEDULE = {true, true, false, true, false, true};

  private final Timer teleopTimer = new Timer();
  private boolean teleopStarted = false;

  private boolean wonAuto = false;
  private boolean wonAutoFromFMS = false;
  private boolean wonAutoOverrideSet = false;
  private boolean wonAutoOverrideValue = false;
  private ScheduleConfidence confidence = ScheduleConfidence.FALLBACK;
  private boolean scheduleLocked = false;

  private final TunableNumber hubTrackingEnabled =
      new TunableNumber("HubShift/TrackingEnabled", 1.0);
  private final TunableNumber featureEnabled = new TunableNumber("HubShift/FireAuthEnabled", 1.0);

  private final TunableNumber fuelCountDelayMin =
      new TunableNumber("HubShift/FuelDelayMinSec", HubTimingConstants.FUEL_COUNT_DELAY_MIN_SEC);
  private final TunableNumber fuelCountDelayMax =
      new TunableNumber("HubShift/FuelDelayMaxSec", HubTimingConstants.FUEL_COUNT_DELAY_MAX_SEC);
  private final TunableNumber fuelCountExtension =
      new TunableNumber("HubShift/FuelExtensionSec", HubTimingConstants.FUEL_COUNT_EXTENSION_SEC);

  private ShiftInfo officialInfo;
  private ShiftInfo shiftedInfo;

  // pre-allocated to avoid GC pressure in the 20ms loop
  private final double[] shiftedStartsBuf = new double[PHASE_STARTS.length];
  private final double[] shiftedEndsBuf = new double[PHASE_ENDS.length];
  private double currentTOF = 0;
  private boolean gameDataMissing = false;

  private HubShiftEngine() {
    officialInfo = new ShiftInfo(ShiftPhase.DISABLED, true, 0, 0, 0, 0);
    shiftedInfo = officialInfo;
  }

  public static HubShiftEngine getInstance() {
    if (instance == null) {
      instance = new HubShiftEngine();
    }
    return instance;
  }

  /** Call at teleop start (from Robot.teleopInit()). */
  public void initializeTeleop() {
    teleopTimer.restart();
    teleopStarted = true;
    scheduleLocked = false;
    parseAutoWinner();
  }

  /** Call every robotPeriodic(). Pass current TOF from ShotCalculator. */
  public void update(double shotTOF) {
    currentTOF = Math.max(0, shotTOF);

    if (!teleopStarted || !DriverStation.isTeleop()) {
      officialInfo =
          new ShiftInfo(
              DriverStation.isAutonomous() ? ShiftPhase.AUTO : ShiftPhase.DISABLED,
              true,
              0,
              0,
              0,
              Double.MAX_VALUE);
      shiftedInfo = officialInfo;
      gameDataMissing = false;
      return;
    }

    if (!scheduleLocked) {
      parseAutoWinner();
    }

    double elapsed = teleopTimer.get();
    boolean[] schedule = wonAuto ? WON_SCHEDULE : LOST_SCHEDULE;

    officialInfo = computeShiftInfo(schedule, PHASE_STARTS, PHASE_ENDS, elapsed);

    if (isFeatureEnabled() && currentTOF > 0) {
      double approachingOffset = -(currentTOF + fuelCountDelayMin.get());
      double endingOffset = fuelCountExtension.get() - (currentTOF + fuelCountDelayMax.get());
      fillShiftedBoundaries(
          shiftedStartsBuf, PHASE_STARTS, schedule, approachingOffset, endingOffset);
      fillShiftedBoundaries(shiftedEndsBuf, PHASE_ENDS, schedule, approachingOffset, endingOffset);
      shiftedInfo = computeShiftInfo(schedule, shiftedStartsBuf, shiftedEndsBuf, elapsed);
    } else {
      shiftedInfo = officialInfo;
    }

    gameDataMissing =
        DriverStation.isFMSAttached()
            && !wonAutoFromFMS
            && !wonAutoOverrideSet
            && elapsed < HubTimingConstants.TRANSITION_END;

    logTelemetry();
  }

  private void parseAutoWinner() {
    if (wonAutoOverrideSet) {
      wonAuto = wonAutoOverrideValue;
      wonAutoFromFMS = false;
      confidence = ScheduleConfidence.LOW;
      scheduleLocked = true;
      return;
    }

    String message = DriverStation.getGameSpecificMessage();
    if (message != null && !message.isEmpty()) {
      char winner = message.charAt(0);
      var ourAlliance = DriverStation.getAlliance();
      if (ourAlliance.isPresent()) {
        boolean redWon = (winner == 'R');
        boolean weAreRed = (ourAlliance.get() == DriverStation.Alliance.Red);
        wonAuto = (redWon == weAreRed);
        wonAutoFromFMS = true;
        confidence = ScheduleConfidence.HIGH;
        scheduleLocked = true;
        return;
      }
    }

    if (!DriverStation.isFMSAttached()) {
      wonAuto = SmartDashboard.getBoolean("WonAuto", false);
      wonAutoFromFMS = false;
      confidence = ScheduleConfidence.LOW;
      return;
    }

    wonAuto = false;
    wonAutoFromFMS = false;
    confidence = ScheduleConfidence.FALLBACK;
  }

  private ShiftInfo computeShiftInfo(
      boolean[] schedule, double[] starts, double[] ends, double elapsed) {
    int phaseIndex = -1;
    for (int i = 0; i < starts.length; i++) {
      if (elapsed >= starts[i] && elapsed < ends[i]) {
        phaseIndex = i;
        break;
      }
    }
    if (phaseIndex < 0) {
      phaseIndex = starts.length - 1;
    }

    ShiftPhase phase = PHASES[phaseIndex];
    boolean hubActive = isHubTrackingEnabled() ? schedule[phaseIndex] : true;
    double elapsedInPhase = elapsed - starts[phaseIndex];
    double remainingInPhase = ends[phaseIndex] - elapsed;

    double remainingInState = remainingInPhase;
    if (phaseIndex < ends.length - 1 && schedule[phaseIndex] == schedule[phaseIndex + 1]) {
      remainingInState = ends[phaseIndex + 1] - elapsed;
    }

    double timeToNextActive = 0;
    if (!hubActive) {
      for (int i = phaseIndex + 1; i < schedule.length; i++) {
        if (schedule[i]) {
          timeToNextActive = starts[i] - elapsed;
          break;
        }
      }
    }

    double timeUntilDeactivation = Double.MAX_VALUE;
    if (hubActive) {
      for (int i = phaseIndex; i < schedule.length; i++) {
        if (!schedule[i] || i == schedule.length - 1) {
          timeUntilDeactivation =
              (i < schedule.length && !schedule[i]) ? starts[i] - elapsed : ends[i] - elapsed;
          break;
        }
      }
    }

    return new ShiftInfo(
        phase,
        hubActive,
        elapsedInPhase,
        remainingInState,
        timeToNextActive,
        timeUntilDeactivation);
  }

  private void fillShiftedBoundaries(
      double[] out,
      double[] boundaries,
      boolean[] schedule,
      double approachingOffset,
      double endingOffset) {
    out[0] = boundaries[0];

    for (int i = 1; i < boundaries.length; i++) {
      boolean prevActive = schedule[Math.min(i - 1, schedule.length - 1)];
      boolean nextActive = schedule[Math.min(i, schedule.length - 1)];

      if (!prevActive && nextActive) {
        out[i] = boundaries[i] + approachingOffset;
      } else if (prevActive && !nextActive) {
        out[i] = boundaries[i] + endingOffset;
      } else {
        out[i] = boundaries[i];
      }
    }
  }

  private void logTelemetry() {
    // competition signals (always logged)
    SafeLog.put("HubShift/Phase", officialInfo.phase().name());
    SafeLog.put("HubShift/OfficialActive", officialInfo.hubActive());
    SafeLog.put("HubShift/TimeToNextActive", officialInfo.timeToNextActive());
    SafeLog.put("HubShift/WonAuto", wonAuto);
    SafeLog.put("HubShift/TimeUntilDeactivation", officialInfo.timeUntilDeactivation());
    SafeLog.put("HubShift/GameDataMissing", gameDataMissing);

    if (Constants.TUNING_MODE) {
      SafeLog.put("HubShift/ShiftedActive", shiftedInfo.hubActive());
      SafeLog.put("HubShift/ElapsedInPhase", officialInfo.elapsedInPhase());
      SafeLog.put("HubShift/RemainingInState", officialInfo.remainingInState());
      SafeLog.put("HubShift/WonAutoFromFMS", wonAutoFromFMS);
      SafeLog.put("HubShift/ScheduleConfidence", confidence.name());
      SafeLog.put("HubShift/TrackingEnabled", isHubTrackingEnabled());
      SafeLog.put("HubShift/CurrentTOF", currentTOF);
    }
  }

  public ShiftInfo getOfficialInfo() {
    return officialInfo;
  }

  public ShiftInfo getShiftedInfo() {
    return shiftedInfo;
  }

  public ScheduleConfidence getConfidence() {
    return confidence;
  }

  public boolean isGameDataMissing() {
    return gameDataMissing;
  }

  public boolean isHubTrackingEnabled() {
    return hubTrackingEnabled.get() >= 0.5;
  }

  public boolean isFeatureEnabled() {
    return featureEnabled.get() >= 0.5;
  }

  public boolean isWonAuto() {
    return wonAuto;
  }

  public boolean isWonAutoFromFMS() {
    return wonAutoFromFMS;
  }

  public void setWonAutoOverride(boolean won) {
    wonAutoOverrideSet = true;
    wonAutoOverrideValue = won;
    wonAuto = won;
    confidence = ScheduleConfidence.LOW;
    scheduleLocked = true;
  }

  public void clearWonAutoOverride() {
    wonAutoOverrideSet = false;
    scheduleLocked = false;
  }

  public double getTeleopElapsed() {
    return teleopStarted ? teleopTimer.get() : 0;
  }

  public double getFuelCountDelayMin() {
    return fuelCountDelayMin.get();
  }

  public double getFuelCountDelayMax() {
    return fuelCountDelayMax.get();
  }

  public double getFuelCountExtension() {
    return fuelCountExtension.get();
  }

  static void resetInstance() {
    instance = null;
  }
}
