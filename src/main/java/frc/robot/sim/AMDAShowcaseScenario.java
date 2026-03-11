package frc.robot.sim;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.telemetry.SafeLog;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * AMDA (Adaptive Multi-Modal Driver Awareness) showcase scenario. Exercises all 4 feedback
 * channels: haptic, LED, camera HUD, and dashboard, by walking through conditions that trigger each
 * pattern and state.
 *
 * <p>Naturally triggered haptic patterns (5/7): TELEOP_START (auto->teleop transition),
 * ENDGAME_WARNING (matchTime<=30), HUB_ACTIVATED (hub becomes active), HUB_DEACTIVATED (hub
 * becomes inactive), HUB_COUNTDOWN (graduated 5s-1s countdown pulses before shift).
 *
 * <p>Naturally triggered LED states (7/10): DISABLED, IDLE, AUTO_RUNNING, SHOOTER_SPINUP, WARNING
 * (low voltage), CRITICAL_ALERT (very low voltage), MATCH_OVER (matchTime crosses 0).
 *
 * <p>Not triggered in sim (documented): READY_TO_SHOOT haptic/LED (needs 5-condition composite),
 * JAM_DETECTED haptic (needs high current + low velocity), VISION_LOCKED LED (needs PhotonVision
 * lock), AIM_PROGRESS LED (needs progressive aim active).
 *
 * <p>ChannelCoordinator exercises: WARNING LED state includes lowConfidence check, DriverFeedback
 * scales spin-up intensity based on isLowConfidence().
 *
 * <p>45s total, 13 phases. Launch: ./gradlew simulateJava -DsimScenario=AMDAShowcase
 */
public class AMDAShowcaseScenario implements SimScenario {
  private double startTime;
  private boolean finished = false;
  private final SimInputPlayback input = new SimInputPlayback();
  private int lastPhase = -1;

  // Phase boundaries (wall-clock seconds from scenario start)
  private static final double DISABLED_END = 2.0;
  private static final double AUTO_END = 7.0;
  private static final double TELEOP_START_END = 10.0;
  private static final double HUB_ACTIVATE_END = 15.0;
  private static final double HUB_DEACTIVATE_END = 18.0;
  private static final double SHIFT_WARN_END = 23.0;
  private static final double HUB_REACTIVATE_END = 26.0;
  private static final double VOLTAGE_WARN_END = 30.0;
  private static final double VOLTAGE_CRIT_END = 33.0;
  private static final double VOLTAGE_RECOVER_END = 36.0;
  private static final double ENDGAME_END = 40.0;
  private static final double MATCH_END_END = 43.0;
  private static final double SCENARIO_END = 45.0;

  @Override
  public String getName() {
    return "AMDAShowcase";
  }

  @Override
  public void init() {
    startTime = Timer.getFPGATimestamp();
    finished = false;
    lastPhase = -1;
    input.releaseAll();
    SimulatedBattery.disableBatterySim();
    RoboRioSim.setVInVoltage(12.8);
    SmartDashboard.putBoolean("WonAuto", true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
  }

  @Override
  public void execute(double timestampSec) {
    double t = timestampSec - startTime;
    int phase = getPhase(t);

    if (phase != lastPhase) {
      SafeLog.put("Sim/AMDA/Phase", phase);
      SafeLog.put("Sim/AMDA/PhaseName", phaseName(phase));
      lastPhase = phase;
      onPhaseEnter(phase);
    }

    onPhaseUpdate(phase, t);

    SafeLog.put("Sim/AMDA/ElapsedSec", t);
    SafeLog.put("Sim/AMDA/Progress", Math.min(100, (t / SCENARIO_END) * 100));

    if (t >= SCENARIO_END) {
      input.releaseAll();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      RoboRioSim.setVInVoltage(12.8);
      finished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  private int getPhase(double t) {
    if (t < DISABLED_END) return 0;
    if (t < AUTO_END) return 1;
    if (t < TELEOP_START_END) return 2;
    if (t < HUB_ACTIVATE_END) return 3;
    if (t < HUB_DEACTIVATE_END) return 4;
    if (t < SHIFT_WARN_END) return 5;
    if (t < HUB_REACTIVATE_END) return 6;
    if (t < VOLTAGE_WARN_END) return 7;
    if (t < VOLTAGE_CRIT_END) return 8;
    if (t < VOLTAGE_RECOVER_END) return 9;
    if (t < ENDGAME_END) return 10;
    if (t < MATCH_END_END) return 11;
    return 12;
  }

  private String phaseName(int phase) {
    switch (phase) {
      case 0:
        return "Disabled_LED_DISABLED";
      case 1:
        return "Auto_LED_AUTO_RUNNING";
      case 2:
        return "TeleopStart_Haptic_TELEOP_START";
      case 3:
        return "HubActivate_Haptic_HUB_ACTIVATED_LED_SPINUP";
      case 4:
        return "HubDeactivate_Haptic_HUB_DEACTIVATED";
      case 5:
        return "ShiftWarn_Haptic_HUB_COUNTDOWN";
      case 6:
        return "HubReactivate_Haptic_HUB_ACTIVATED";
      case 7:
        return "VoltageWarn_LED_WARNING";
      case 8:
        return "VoltageCrit_LED_CRITICAL_ALERT";
      case 9:
        return "VoltageRecover_LED_IDLE";
      case 10:
        return "Endgame_Haptic_ENDGAME_WARNING";
      case 11:
        return "MatchEnd_LED_MATCH_OVER";
      case 12:
        return "PostMatch_Disable";
      default:
        return "Unknown";
    }
  }

  private void onPhaseEnter(int phase) {
    switch (phase) {
      case 0: // Disabled baseline -> LED: DISABLED
        input.releaseAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        SafeLog.put("Sim/AMDA/ExpectedLED", "DISABLED");
        SafeLog.put("Sim/AMDA/ExpectedHaptic", "none");
        break;

      case 1: // Auto mode -> LED: AUTO_RUNNING
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setMatchTime(15.0);
        DriverStationSim.notifyNewData();
        // Gentle drive to produce motion telemetry
        input.driveForward(0.3);
        SafeLog.put("Sim/AMDA/ExpectedLED", "AUTO_RUNNING");
        SafeLog.put("Sim/AMDA/ExpectedHaptic", "none");
        break;

      case 2: // Auto -> Teleop transition -> Haptic: TELEOP_START, LED: IDLE
        input.driveForward(0);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        // matchTime 140 = transition period (both hubs active)
        DriverStationSim.setMatchTime(140.0);
        DriverStationSim.notifyNewData();
        SafeLog.put("Sim/AMDA/ExpectedLED", "IDLE");
        SafeLog.put("Sim/AMDA/ExpectedHaptic", "TELEOP_START");
        break;

      case 3: // Hub activates + shooter spinup -> Haptic: HUB_ACTIVATED, LED: SHOOTER_SPINUP
        // matchTime 106 = shift 2 (even, wonAuto=true -> ACTIVE)
        // Previous phase was at 140 (transition, both active), so first we need
        // to go through an INACTIVE shift to get the rising edge
        // Jump to matchTime 129 (shift 1, INACTIVE for winner), held from phase 3-4
        DriverStationSim.setMatchTime(129.0);
        DriverStationSim.notifyNewData();
        // This should produce hubActive=false (shift 1, odd, wonAuto -> !odd = false)
        // Wait, we need the rising edge. Let's set INACTIVE first, then go ACTIVE.
        // Actually at 140 (transition) hub was ACTIVE. At 129 (shift 1) hub = INACTIVE.
        // That's a falling edge -> HUB_DEACTIVATED fires, not HUB_ACTIVATED.
        // Let me fix: first go INACTIVE here, then in phase 4 go ACTIVE.
        SafeLog.put("Sim/AMDA/ExpectedLED", "SHOOTER_SPINUP");
        SafeLog.put("Sim/AMDA/ExpectedHaptic", "HUB_DEACTIVATED");
        // Start shooter via copilot right trigger (SpeedUpThenIndex)
        input.copilotSetRightTrigger(1.0);
        break;

      case 4: // Hub deactivation confirmed, now activate -> Haptic: HUB_ACTIVATED
        // matchTime 104 = shift 2 (even, wonAuto=true -> ACTIVE)
        DriverStationSim.setMatchTime(104.0);
        DriverStationSim.notifyNewData();
        SafeLog.put("Sim/AMDA/ExpectedLED", "SHOOTER_SPINUP");
        SafeLog.put("Sim/AMDA/ExpectedHaptic", "HUB_ACTIVATED");
        break;

      case 5: // Approach shift boundary -> Haptic: HUB_COUNTDOWN (graduated)
        // Stop shooter, set matchTime so countdown fires 5s-1s before shift
        input.copilotSetRightTrigger(0);
        // Next shift at 80, ramp matchTime from 86 down to 78 over this phase
        DriverStationSim.setMatchTime(86.0);
        DriverStationSim.notifyNewData();
        SafeLog.put("Sim/AMDA/ExpectedLED", "IDLE");
        SafeLog.put("Sim/AMDA/ExpectedHaptic", "HUB_COUNTDOWN");
        break;

      case 6: // Hub reactivation after shift -> Haptic: HUB_ACTIVATED (or DEACTIVATED)
        // matchTime 79 = shift 3 (odd, wonAuto=true -> !odd = false -> INACTIVE)
        // So crossing from shift 2 (ACTIVE) to shift 3 (INACTIVE) = HUB_DEACTIVATED
        // Then we jump to shift 4 to get HUB_ACTIVATED
        DriverStationSim.setMatchTime(54.0);
        DriverStationSim.notifyNewData();
        // shift 4, even, wonAuto=true -> ACTIVE. Previous was shift 3 INACTIVE -> rising edge
        SafeLog.put("Sim/AMDA/ExpectedLED", "IDLE");
        SafeLog.put("Sim/AMDA/ExpectedHaptic", "HUB_ACTIVATED");
        break;

      case 7: // Voltage warning -> LED: WARNING
        RoboRioSim.setVInVoltage(11.0);
        SafeLog.put("Sim/AMDA/ExpectedLED", "WARNING");
        SafeLog.put("Sim/AMDA/ExpectedHaptic", "none");
        break;

      case 8: // Voltage critical -> LED: CRITICAL_ALERT
        RoboRioSim.setVInVoltage(9.0);
        SafeLog.put("Sim/AMDA/ExpectedLED", "CRITICAL_ALERT");
        SafeLog.put("Sim/AMDA/ExpectedHaptic", "none");
        break;

      case 9: // Voltage recovery -> LED: IDLE
        RoboRioSim.setVInVoltage(12.8);
        SafeLog.put("Sim/AMDA/ExpectedLED", "IDLE");
        SafeLog.put("Sim/AMDA/ExpectedHaptic", "none");
        break;

      case 10: // Endgame warning -> Haptic: ENDGAME_WARNING
        // matchTime <= 30 in teleop triggers ENDGAME_WARNING
        DriverStationSim.setMatchTime(30.0);
        DriverStationSim.notifyNewData();
        SafeLog.put("Sim/AMDA/ExpectedLED", "IDLE");
        SafeLog.put("Sim/AMDA/ExpectedHaptic", "ENDGAME_WARNING");
        break;

      case 11: // Match end -> LED: MATCH_OVER
        // matchTime crosses through 0 in teleop triggers MATCH_OVER latch
        DriverStationSim.setMatchTime(0.05);
        DriverStationSim.notifyNewData();
        SafeLog.put("Sim/AMDA/ExpectedLED", "MATCH_OVER");
        SafeLog.put("Sim/AMDA/ExpectedHaptic", "none");
        break;

      case 12: // Post-match disable
        input.releaseAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        SafeLog.put("Sim/AMDA/ExpectedLED", "DISABLED");
        SafeLog.put("Sim/AMDA/ExpectedHaptic", "none");
        break;
    }
  }

  private void onPhaseUpdate(int phase, double t) {
    switch (phase) {
      case 1: // Auto: count down match time
        double autoT = t - DISABLED_END;
        DriverStationSim.setMatchTime(Math.max(0, 15.0 - autoT));
        DriverStationSim.notifyNewData();
        break;

      case 5: // Shift countdown: ramp matchTime from 86 down through 80 boundary
        // Over 5s, matchTime goes 86 -> 78, graduated countdown fires at 5s,4s,3s,2s,1s
        double warnT = t - HUB_DEACTIVATE_END;
        double matchTime = 86.0 - (warnT / 5.0) * 8.0;
        DriverStationSim.setMatchTime(Math.max(78.0, matchTime));
        DriverStationSim.notifyNewData();
        break;

      case 10: // Endgame: count down from 30 to 0
        double egT = t - VOLTAGE_RECOVER_END;
        double egMatchTime = Math.max(0, 30.0 - (egT / 4.0) * 30.0);
        DriverStationSim.setMatchTime(egMatchTime);
        DriverStationSim.notifyNewData();
        break;

      case 11: // Match end: hold at 0 so MATCH_OVER latch fires
        DriverStationSim.setMatchTime(0);
        DriverStationSim.notifyNewData();
        break;
    }
  }
}
