package frc.robot.sim;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.SpeedUpThenIndex;
import frc.robot.util.SafeLog;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Definitive competition match with hub-aware strategy.
 *
 * <p>Starts from Red alliance wall (15, 4, 180deg) and plays a full 169s match with
 * game-manual-accurate hub shift timing.
 *
 * <p>Hub shift logic (WonAuto=true, Red Alliance): Transition shift (matchTime 140-130): ACTIVE
 * Shift 1 (130-105): INACTIVE,collect fuel, reposition Shift 2 (105-80): ACTIVE,rapid-fire scoring
 * dump Shift 3 (80-55): INACTIVE,collect from far zones Shift 4 (55-30): ACTIVE, aggressive scoring
 * Endgame (<=30): ACTIVE,final shots + hanger climb
 *
 * <p>Strategy: ACTIVE shifts keep SpeedUpThenIndex running continuously (never stop/restart within
 * a phase). INACTIVE shifts collect fuel with intake, zero shooting.
 *
 * <p>Drive mapping (Red alliance + allianceRelativeControl): driveForward(+) = toward BLUE side
 * (field -X) = toward fuel zones driveForward(-) = toward RED wall (field +X) = toward hub/tower
 *
 * <p>Field layout: Red start: (15.0, 5.5, 180deg),near Red wall, offset Y above hub Red Hub: (11.9,
 * 4.0),4 hex faces, 1.06m opening Center fuel: ~(8.27, 4.0), upper: (8.0, 5.5), lower: (8.5, 2.5)
 * Red Tower: ~(15.5, 4.0),3 rungs for climbing
 *
 * <p>Scoring: X-only (SpeedUpThenIndex). NEVER A+X,command conflict. 169s scenario, 11 phases
 * (0-10), ~75-110 shots. Launch: ./gradlew simulateJava -DsimScenario=CompetitionMatch
 */
class CompetitionMatchScenario implements SimScenario {
  private double startTime;
  private boolean finished = false;
  private final SimInputPlayback input = new SimInputPlayback();
  private int lastPhase = -1;

  // Direct command control (bypasses trigger system)
  private Command activeShooterCmd = null;

  // Wall-clock phase boundaries aligned with hub shift matchTime boundaries
  private static final double PRE_MATCH_END = 3.0;
  private static final double AUTO_END = 23.0; // 20s auto
  private static final double DISABLED_END = 26.0; // 3s disabled transition
  private static final double TRANS_SHIFT_END = 36.0; // 10s transition shift (ACTIVE)
  private static final double SHIFT1_END = 61.0; // 25s shift 1 (INACTIVE)
  private static final double SHIFT2_END = 86.0; // 25s shift 2 (ACTIVE)
  private static final double SHIFT3_END = 111.0; // 25s shift 3 (INACTIVE)
  private static final double SHIFT4_END = 136.0; // 25s shift 4 (ACTIVE)
  private static final double ENDGAME_SCORE_END = 151.0; // 15s endgame scoring (ACTIVE)
  private static final double HANGER_END = 166.0; // 15s hanger climb
  private static final double SCENARIO_END = 169.0; // 3s post-match

  @Override
  public String getName() {
    return "CompetitionMatch";
  }

  @Override
  public void init() {
    startTime = Timer.getFPGATimestamp();
    finished = false;
    lastPhase = -1;
    activeShooterCmd = null;
    input.releaseAll();
    SimulatedBattery.disableBatterySim(); // prevent MapleSim from overwriting our voltage ramp
    RoboRioSim.setVInVoltage(12.8);
    SmartDashboard.putBoolean("WonAuto", false);
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();

    // Teleport robot to Red alliance wall starting position
    try {
      RobotContainer rc = RobotContainer.getInstance();
      if (rc != null) {
        Pose2d startPose = new Pose2d(new Translation2d(15.0, 5.5), Rotation2d.fromDegrees(180));
        rc.getSwerveSubsystem().resetOdometry(startPose);
      }
    } catch (RuntimeException e) {
      SafeLog.put("Sim/CompMatch/InitError", e.getClass().getSimpleName());
    }
  }

  @Override
  public void execute(double timestampSec) {
    double t = timestampSec - startTime;
    int phase = getPhase(t);

    if (phase != lastPhase) {
      SafeLog.put("Sim/CompMatch/Phase", phase);
      SafeLog.put("Sim/CompMatch/PhaseName", phaseName(phase));
      lastPhase = phase;
      onPhaseEnter(phase);
    }

    onPhaseUpdate(phase, t);

    SafeLog.put("Sim/CompMatch/ElapsedSec", t);
    SafeLog.put("Sim/CompMatch/Progress", Math.min(100, (t / SCENARIO_END) * 100));

    if (t >= SCENARIO_END) {
      input.releaseAll();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      finished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  private int getPhase(double t) {
    if (t < PRE_MATCH_END) return 0;
    if (t < AUTO_END) return 1;
    if (t < DISABLED_END) return 2;
    if (t < TRANS_SHIFT_END) return 3;
    if (t < SHIFT1_END) return 4;
    if (t < SHIFT2_END) return 5;
    if (t < SHIFT3_END) return 6;
    if (t < SHIFT4_END) return 7;
    if (t < ENDGAME_SCORE_END) return 8;
    if (t < HANGER_END) return 9;
    return 10;
  }

  private String phaseName(int phase) {
    switch (phase) {
      case 0:
        return "PreMatch";
      case 1:
        return "Autonomous";
      case 2:
        return "AutoTeleopDisabled";
      case 3:
        return "TransitionShift_ACTIVE";
      case 4:
        return "Shift1_INACTIVE";
      case 5:
        return "Shift2_ACTIVE";
      case 6:
        return "Shift3_INACTIVE";
      case 7:
        return "Shift4_ACTIVE";
      case 8:
        return "EndgameScoring_ACTIVE";
      case 9:
        return "HangerClimb";
      case 10:
        return "PostMatch";
      default:
        return "Unknown";
    }
  }

  /**
   * Phase entry: set mode flags and start/stop shooting. Shooting is managed at phase boundaries
   * ONLY,never within a phase.
   */
  private void onPhaseEnter(int phase) {
    // Clear drive inputs,shooting handled per-phase below
    input.driveForward(0);
    input.strafe(0);
    input.rotate(0);
    input.holdB(false);
    input.holdRightBumper(false);

    switch (phase) {
      case 0: // Pre-match disabled
        input.releaseAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        break;
      case 1: // Auto
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setMatchTime(20.0);
        DriverStationSim.notifyNewData();
        // Shooting started inside autoPhase after sprint begins
        break;
      case 2: // Disabled transition
        stopShooting();
        input.releaseAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        break;
      case 3: // Transition shift ACTIVE
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        SmartDashboard.putBoolean("WonAuto", true);
        DriverStationSim.setMatchTime(140.0);
        DriverStationSim.notifyNewData();
        startShooting();
        break;
      case 4: // Shift 1 INACTIVE
        stopShooting();
        break;
      case 5: // Shift 2 ACTIVE
        startShooting();
        break;
      case 6: // Shift 3 INACTIVE
        stopShooting();
        break;
      case 7: // Shift 4 ACTIVE
        startShooting();
        break;
      case 8: // Endgame ACTIVE
        startShooting();
        break;
      case 9: // Hanger
        stopShooting();
        break;
      case 10: // Post-match
        stopShooting();
        input.releaseAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        break;
    }
  }

  private void onPhaseUpdate(int phase, double t) {
    // Match time countdown
    if (phase == 1) {
      DriverStationSim.setMatchTime(Math.max(0, 20.0 - (t - PRE_MATCH_END)));
      DriverStationSim.notifyNewData();
    } else if (phase >= 3 && phase <= 9) {
      double matchTime = 140.0 - (t - DISABLED_END);
      DriverStationSim.setMatchTime(Math.max(0, matchTime));
      DriverStationSim.notifyNewData();
    }

    // Battery sag: 12.8V -> 11.2V over teleop
    if (phase >= 3 && phase <= 9) {
      double pct = (t - DISABLED_END) / (HANGER_END - DISABLED_END);
      RoboRioSim.setVInVoltage(Math.max(11.2, 12.8 - pct * 1.6));
    }

    // Log hub state for analysis (auto counts as ACTIVE too)
    boolean hubActive = (phase == 1 || phase == 3 || phase == 5 || phase == 7 || phase == 8);
    SafeLog.put("Sim/CompMatch/HubActive", hubActive);

    switch (phase) {
      case 1:
        autoPhase(t);
        break;
      case 3:
        transitionShiftPhase(t);
        break;
      case 4:
        shift1InactivePhase(t);
        break;
      case 5:
        shift2ActivePhase(t);
        break;
      case 6:
        shift3InactivePhase(t);
        break;
      case 7:
        shift4ActivePhase(t);
        break;
      case 8:
        endgameScoringPhase(t);
        break;
      case 9:
        hangerPhase(t);
        break;
    }
  }

  /**
   * AUTO (3-23s, 20s): Sprint from Red wall to hub, shoot, sprint to center fuel, intake, sprint
   * back to hub, shoot more.
   *
   * <p>Starting at (15, 5.5). Hub at (11.9, 4),1.5m Y offset clears it. Center fuel at (8.27, 4) =
   * 6.7m in +forward direction.
   */
  private void autoPhase(double t) {
    double at = t - PRE_MATCH_END;

    if (at < 2.0) {
      // Sprint from Red wall toward hub (Y=5.5 naturally clears hub at Y=4)
      input.driveForward(0.95);
      input.strafe(0.15);
      startShooting();
    } else if (at < 5.0) {
      // Past hub area, shooting + drift toward center Y
      input.driveForward(0.1);
      input.strafe(-0.15 + 0.1 * Math.sin(at * 2.0));
    } else if (at < 8.0) {
      // Sprint to center fuel, deploy intake
      input.driveForward(0.9);
      input.strafe(0.2);
      input.holdB(true);
    } else if (at < 10.0) {
      // Intake while creeping through fuel zone
      input.holdB(true);
      input.driveForward(0.2);
      input.strafe(0.1);
    } else if (at < 14.0) {
      // Sprint back toward hub area
      input.holdB(false);
      input.driveForward(-0.9);
      input.strafe(-0.25);
    } else {
      // Park near hub scoring face
      input.driveForward(-0.05);
      input.strafe(-0.08 + 0.1 * Math.sin(at * 1.5));
    }
  }

  /**
   * TRANSITION SHIFT ACTIVE (26-36s, 10s): Hub ACTIVE. Shoot from hub position, quick fuel grab,
   * continue shooting on return. Shooter runs continuously,started at phase enter.
   */
  private void transitionShiftPhase(double t) {
    double pt = t - DISABLED_END;

    if (pt < 3.0) {
      // Near hub from auto, shoot while strafing
      input.driveForward(-0.1);
      input.strafe(0.1 * Math.sin(pt * 2.5));
    } else if (pt < 6.0) {
      // Quick fuel grab toward center field
      input.driveForward(0.85);
      input.strafe(-0.3);
      input.holdB(true);
    } else if (pt < 9.0) {
      // Sprint back to hub area
      input.holdB(false);
      input.driveForward(-0.85);
      input.strafe(0.25);
    } else {
      // At hub scoring face
      input.driveForward(-0.05);
      input.strafe(0.08 * Math.sin(pt * 2.0));
    }
  }

  /**
   * SHIFT 1 INACTIVE (36-61s, 25s): Hub INACTIVE,NO SHOOTING. 3 fuel collection runs across
   * different field zones. Ends repositioned near hub for shift 2 activation.
   */
  private void shift1InactivePhase(double t) {
    double pt = t - TRANS_SHIFT_END;

    if (pt < 8.0) {
      // Run 1: Center fuel, go above hub (y > 5)
      fuelRun(pt, 0.9, 0.8, -0.4);
    } else if (pt < 16.0) {
      // Run 2: Upper far fuel zone
      fuelRun(pt - 8.0, 0.9, 0.9, -0.5);
    } else if (pt < 22.0) {
      // Run 3: Lower fuel, go below hub (y < 3)
      fuelRun(pt - 16.0, 0.85, -0.9, 0.5);
    } else {
      // Reposition near hub scoring face
      input.holdB(false);
      input.driveForward(-0.7);
      input.strafe(0.3);
    }
  }

  /**
   * SHIFT 2 ACTIVE (61-86s, 25s): Hub ACTIVE,rapid-fire scoring. Shooter runs continuously. 2 drive
   * cycles sprint to fuel and back while shooter fires the whole time.
   */
  private void shift2ActivePhase(double t) {
    double pt = t - SHIFT1_END;

    if (pt < 3.0) {
      // Immediate dump from pre-position, strafe for angle
      input.driveForward(-0.1);
      input.strafe(0.15 * Math.sin(pt * 3.0));
    } else if (pt < 13.0) {
      // Cycle 1: go above hub to fuel
      activeDriveCycle(pt - 3.0, 0.9, 0.8, -0.5);
    } else if (pt < 23.0) {
      // Cycle 2: go below hub to fuel
      activeDriveCycle(pt - 13.0, 0.9, -0.8, 0.5);
    } else {
      // Park near hub
      input.driveForward(-0.05);
      input.strafe(0.1 * Math.sin(pt * 2.0));
    }
  }

  /**
   * SHIFT 3 INACTIVE (86-111s, 25s): Hub INACTIVE,NO SHOOTING. Deep fuel collection with wider
   * field coverage.
   */
  private void shift3InactivePhase(double t) {
    double pt = t - SHIFT2_END;

    if (pt < 8.0) {
      // Run 1: Deep neutral, go below hub
      fuelRun(pt, 0.95, -0.8, 0.5);
    } else if (pt < 16.0) {
      // Run 2: Upper far fuel
      fuelRun(pt - 8.0, 0.95, 0.9, -0.4);
    } else if (pt < 22.0) {
      // Run 3: Lower far fuel
      fuelRun(pt - 16.0, 0.90, -0.9, 0.5);
    } else {
      // Reposition near hub
      input.holdB(false);
      input.driveForward(-0.75);
      input.strafe(-0.35);
    }
  }

  /**
   * SHIFT 4 ACTIVE (111-136s, 25s): Hub ACTIVE,aggressive scoring. Max speed cycles, battery
   * sagging. Shooter runs continuously.
   */
  private void shift4ActivePhase(double t) {
    double pt = t - SHIFT3_END;

    if (pt < 3.0) {
      // Immediate dump
      input.driveForward(-0.1);
      input.strafe(-0.15 * Math.sin(pt * 3.0));
    } else if (pt < 13.0) {
      // Cycle 1: go below hub to fuel, fast
      activeDriveCycle(pt - 3.0, 0.95, -0.9, 0.6);
    } else if (pt < 23.0) {
      // Cycle 2: go above hub to fuel
      activeDriveCycle(pt - 13.0, 0.95, 0.8, -0.5);
    } else {
      // Park near hub
      input.driveForward(-0.05);
      input.strafe(0.08 * Math.sin(pt * 2.5));
    }
  }

  /**
   * ENDGAME ACTIVE (136-151s, 15s): Both hubs ACTIVE. One quick drive cycle then park at hub and
   * dump.
   */
  private void endgameScoringPhase(double t) {
    double pt = t - SHIFT4_END;

    if (pt < 8.0) {
      // Quick cycle,go above hub
      activeDriveCycle(pt, 0.9, 0.8, -0.5);
    } else {
      // Park at hub and dump
      double ct = pt - 8.0;
      if (ct < 2.0) {
        input.driveForward(-0.85);
        input.strafe(0.3);
      } else {
        input.driveForward(0);
        input.strafe(0.08 * Math.sin(ct * 2.5));
      }
    }
  }

  /**
   * HANGER CLIMB (151-166s, 15s): Sprint to Red Tower, deploy, climb. Tower near (15.5, 4.0),
   * sprint forward (toward Red wall).
   */
  private void hangerPhase(double t) {
    double pt = t - ENDGAME_SCORE_END;

    if (pt < 3.5) {
      // Sprint to tower (-fwd = toward Red wall)
      input.driveForward(-0.95);
      input.strafe(0);
    } else if (pt < 4.5) {
      input.driveForward(0);
    } else if (pt < 11.5) {
      // Deploy hanger
      input.holdRightBumper(true);
    } else {
      // Release triggers ClimbHanger (onFalse)
      input.holdRightBumper(false);
    }
  }

  /**
   * Fuel collection run (~8s) for INACTIVE shifts. Sprint to fuel (+fwd), intake, sprint partway
   * back (-fwd). NO SHOOTING.
   */
  private void fuelRun(double ct, double speed, double outBias, double returnBias) {
    if (ct < 3.0) {
      // Sprint to fuel zone, gentle strafe (start Y=5.5 already clears hub)
      input.driveForward(speed);
      input.strafe(speed * 0.4 * outBias);
    } else if (ct < 5.5) {
      // Intake while creeping
      input.holdB(true);
      input.driveForward(0.25);
      input.strafe(0.1 * outBias);
    } else {
      // Sprint partway back toward hub area
      input.holdB(false);
      input.driveForward(-speed * 0.8);
      input.strafe(speed * 0.3 * returnBias);
    }
  }

  /**
   * Drive cycle during ACTIVE shift (~10s). Shooter stays running,no startShooting/stopShooting
   * calls. Just drive and intake. Sprint to fuel (+fwd), intake, sprint to hub (-fwd), strafe.
   */
  private void activeDriveCycle(double ct, double speed, double outBias, double returnBias) {
    if (ct < 2.5) {
      // Sprint to fuel zone, gentle strafe (start Y=5.5 clears hub)
      input.driveForward(speed);
      input.strafe(speed * 0.4 * outBias);
    } else if (ct < 4.5) {
      // Intake while creeping
      input.holdB(true);
      input.driveForward(0.25);
      input.strafe(0.1 * outBias);
    } else if (ct < 7.0) {
      // Sprint back to hub scoring face
      input.holdB(false);
      input.driveForward(-speed);
      input.strafe(speed * 0.3 * returnBias);
    } else {
      // Near hub, strafe for shot angle variety
      input.driveForward(-0.1);
      input.strafe(0.12 * Math.sin(ct * 2.5));
    }
  }

  private void startShooting() {
    if (activeShooterCmd == null) {
      try {
        activeShooterCmd = new SpeedUpThenIndex();
        CommandScheduler.getInstance().schedule(activeShooterCmd);
      } catch (RuntimeException e) {
        activeShooterCmd = null;
      }
    }
  }

  private void stopShooting() {
    if (activeShooterCmd != null) {
      try {
        CommandScheduler.getInstance().cancel(activeShooterCmd);
      } catch (RuntimeException ignored) {
      }
      activeShooterCmd = null;
    }
  }
}
