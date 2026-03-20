package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HubTimingConstants;
import frc.robot.telemetry.SafeLog;

/**
 * Hub shift timing with TOF-compensated boundaries. Tracks which alliance owns the hub across 4
 * shifts + endgame, with auto-winner detection and copilot override.
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

  // shift boundaries: elapsed seconds from teleop start
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

  // active schedule lookup: [transition, shift1, shift2, shift3, shift4, endgame]
  // won auto = our hub was active first, so it goes inactive in shift1
  private static final boolean[] WON_SCHEDULE = {true, false, true, false, true, true};
  private static final boolean[] LOST_SCHEDULE = {true, true, false, true, false, true};

  private final Timer teleopTimer = new Timer();
  private boolean teleopStarted = false;

  // auto-winner state
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

  /** Call from Robot.teleopInit(). */
  public void initializeTeleop() {
    teleopTimer.restart();
    teleopStarted = true;
    scheduleLocked = false;
    parseAutoWinner();
  }

  /** Call every robotPeriodic(). Pass current TOF from ShotCalculator (0 if not available). */
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
      double[] shiftedStarts =
          computeShiftedBoundaries(PHASE_STARTS, schedule, approachingOffset, endingOffset);
      double[] shiftedEnds =
          computeShiftedBoundaries(PHASE_ENDS, schedule, approachingOffset, endingOffset);
      shiftedInfo = computeShiftInfo(schedule, shiftedStarts, shiftedEnds, elapsed);
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

    // merge consecutive same-state windows for remaining time
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

  /**
   * Shifts boundaries based on whether the transition is into or out of an active window. At
   * inactive->active transitions the boundary shifts earlier so the robot fires before the official
   * start. At active->inactive transitions the boundary shifts by the ending offset.
   */
  private double[] computeShiftedBoundaries(
      double[] boundaries, boolean[] schedule, double approachingOffset, double endingOffset) {
    double[] shifted = new double[boundaries.length];
    shifted[0] = boundaries[0];

    for (int i = 1; i < boundaries.length; i++) {
      boolean prevActive = schedule[Math.min(i - 1, schedule.length - 1)];
      boolean nextActive = schedule[Math.min(i, schedule.length - 1)];

      if (!prevActive && nextActive) {
        shifted[i] = boundaries[i] + approachingOffset;
      } else if (prevActive && !nextActive) {
        shifted[i] = boundaries[i] + endingOffset;
      } else {
        shifted[i] = boundaries[i];
      }
    }
    return shifted;
  }

  private void logTelemetry() {
    SafeLog.put("Scoring/HubShift/Phase", officialInfo.phase().name());
    SafeLog.put("Scoring/HubShift/OfficialActive", officialInfo.hubActive());
    SafeLog.put("Scoring/HubShift/ShiftedActive", shiftedInfo.hubActive());
    SafeLog.put("Scoring/HubShift/ElapsedInPhase", officialInfo.elapsedInPhase());
    SafeLog.put("Scoring/HubShift/RemainingInState", officialInfo.remainingInState());
    SafeLog.put("Scoring/HubShift/TimeToNextActive", officialInfo.timeToNextActive());
    SafeLog.put("Scoring/HubShift/TimeUntilDeactivation", officialInfo.timeUntilDeactivation());
    SafeLog.put("Scoring/HubShift/WonAuto", wonAuto);
    SafeLog.put("Scoring/HubShift/WonAutoFromFMS", wonAutoFromFMS);
    SafeLog.put("Scoring/HubShift/ScheduleConfidence", confidence.name());
    SafeLog.put("Scoring/HubShift/GameDataMissing", gameDataMissing);
    SafeLog.put("Scoring/HubShift/TrackingEnabled", isHubTrackingEnabled());
    SafeLog.put("Scoring/HubShift/CurrentTOF", currentTOF);
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

  /** Copilot sets this via dashboard or button press to override auto-winner detection. */
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
