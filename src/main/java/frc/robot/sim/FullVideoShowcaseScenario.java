package frc.robot.sim;

import static frc.robot.Constants.HubScoringConstants.*;

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
import frc.robot.commands.HubArcDrive;
import frc.robot.commands.SpeedUpThenIndex;
import frc.robot.telemetry.SafeLog;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Full 169s match for video recording that maximizes telemetry coverage.
 *
 * <p>Merges CompetitionMatch (full match structure, hub shifts, hanger climb) with VideoDemo
 * (HubArcDrive orbit during active shifts).
 *
 * <p>Stays on the Red alliance side,no bump/ramp crossing (MapleSim collision body blocks the path,
 * see Arena2026Rebuilt.java).
 *
 * <p>Hub shift logic (WonAuto=true, Red Alliance): Transition shift (matchTime 140-130): ACTIVE,
 * close arc orbit Shift 1 (130-105): INACTIVE,3 fuel collection runs Shift 2 (105-80): ACTIVE, far
 * arc orbit (visual variety) Shift 3 (80-55): INACTIVE,3 fuel collection runs Shift 4 (55-30):
 * ACTIVE,close arc orbit Endgame (<=30): ACTIVE,far arc + final dump
 *
 * <p>Active shifts alternate close (1.06m) and far (2.0m) arc distances. Inactive shifts do fuel
 * collection runs with intake deploy/retract.
 *
 * <p>Drive mapping (Red alliance + allianceRelativeControl): driveForward(+) = toward BLUE side
 * (field -X) = toward fuel driveForward(-) = toward RED wall (field +X) = toward hub strafe(+) =
 * field +Y = away from centerline
 *
 * <p>Launch: ./gradlew simulateJava -DsimScenario=FullVideoShowcase
 */
public class FullVideoShowcaseScenario implements SimScenario {
  private double startTime;
  private boolean finished = false;
  private final SimInputPlayback input = new SimInputPlayback();
  private int lastPhase = -1;

  private Command arcDriveCmd = null;
  private Command shooterCmd = null;

  private static final double CLOSE_ARC = SCORING_DISTANCE; // 1.06m
  private static final double FAR_ARC = 2.0;

  // Phase boundaries,matches CompetitionMatch hub shift timing
  private static final double PRE_MATCH_END = 3.0;
  private static final double AUTO_END = 23.0; // 20s auto
  private static final double DISABLED_END = 26.0; // 3s disabled transition
  private static final double TRANS_SHIFT_END = 36.0; // 10s transition (ACTIVE)
  private static final double SHIFT1_END = 61.0; // 25s shift 1 (INACTIVE)
  private static final double SHIFT2_END = 86.0; // 25s shift 2 (ACTIVE)
  private static final double SHIFT3_END = 111.0; // 25s shift 3 (INACTIVE)
  private static final double SHIFT4_END = 136.0; // 25s shift 4 (ACTIVE)
  private static final double ENDGAME_SCORE_END = 151.0; // 15s endgame (ACTIVE)
  private static final double HANGER_END = 166.0; // 15s hanger climb
  private static final double SCENARIO_END = 169.0; // 3s post-match

  @Override
  public String getName() {
    return "FullVideoShowcase";
  }

  @Override
  public void init() {
    startTime = Timer.getFPGATimestamp();
    finished = false;
    lastPhase = -1;
    arcDriveCmd = null;
    shooterCmd = null;
    input.releaseAll();

    SimulatedBattery.disableBatterySim();
    RoboRioSim.setVInVoltage(12.8);
    SmartDashboard.putBoolean("WonAuto", false);
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();

    // Red alliance start,Y=5.5 naturally clears hub at Y=4.0
    try {
      RobotContainer rc = RobotContainer.getInstance();
      if (rc != null) {
        Pose2d startPose = new Pose2d(new Translation2d(15.0, 5.5), Rotation2d.fromDegrees(180));
        rc.getSwerveSubsystem().resetOdometry(startPose);
      }
    } catch (RuntimeException e) {
      SafeLog.put("Sim/FullShowcase/InitError", e.getClass().getSimpleName());
    }
  }

  @Override
  public void execute(double timestampSec) {
    double t = timestampSec - startTime;
    int phase = getPhase(t);

    if (phase != lastPhase) {
      SafeLog.put("Sim/FullShowcase/Phase", phase);
      SafeLog.put("Sim/FullShowcase/PhaseName", phaseName(phase));
      lastPhase = phase;
      onPhaseEnter(phase);
    }

    onPhaseUpdate(phase, t);

    SafeLog.put("Sim/FullShowcase/ElapsedSec", t);
    SafeLog.put("Sim/FullShowcase/Progress", Math.min(100, (t / SCENARIO_END) * 100));

    // Debug trace every ~1s via telemetry (not println)
    if (((int) (t * 50)) % 50 == 0) {
      double matchTime = phase == 1 ? 20.0 - (t - PRE_MATCH_END) : 140.0 - (t - DISABLED_END);
      SafeLog.put("Sim/FullShowcase/MatchTime", matchTime);
    }

    if (t >= SCENARIO_END) {
      stopAll();
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
        return "Auto";
      case 2:
        return "Disabled";
      case 3:
        return "TransShift_ACTIVE";
      case 4:
        return "Shift1_INACTIVE";
      case 5:
        return "Shift2_ACTIVE";
      case 6:
        return "Shift3_INACTIVE";
      case 7:
        return "Shift4_ACTIVE";
      case 8:
        return "Endgame_ACTIVE";
      case 9:
        return "HangerClimb";
      case 10:
        return "PostMatch";
      default:
        return "Unknown";
    }
  }

  private void onPhaseEnter(int phase) {
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

      case 1: // Autonomous
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setMatchTime(20.0);
        DriverStationSim.notifyNewData();
        break;

      case 2: // Disabled transition
        stopAll();
        input.releaseAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        break;

      case 3: // Transition shift ACTIVE,close arc
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        SmartDashboard.putBoolean("WonAuto", true);
        DriverStationSim.setMatchTime(140.0);
        DriverStationSim.notifyNewData();
        startArcDrive(CLOSE_ARC);
        startShooting();
        break;

      case 4: // Shift 1 INACTIVE,fuel collection
        stopAll();
        break;

      case 5: // Shift 2 ACTIVE,far arc (visual variety)
        startArcDrive(FAR_ARC);
        startShooting();
        break;

      case 6: // Shift 3 INACTIVE,fuel collection
        stopAll();
        break;

      case 7: // Shift 4 ACTIVE,close arc
        startArcDrive(CLOSE_ARC);
        startShooting();
        break;

      case 8: // Endgame ACTIVE,far arc
        startArcDrive(FAR_ARC);
        startShooting();
        break;

      case 9: // Hanger climb
        stopAll();
        break;

      case 10: // Post-match
        stopAll();
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

    // Battery sag: 12.8V -> 11.2V over teleop (steeper than VideoDemo)
    if (phase >= 3 && phase <= 9) {
      double pct = (t - DISABLED_END) / (HANGER_END - DISABLED_END);
      RoboRioSim.setVInVoltage(Math.max(11.2, 12.8 - pct * 1.6));
    }

    switch (phase) {
      case 1:
        autoPhase(t);
        break;
      case 3:
        activeArcPhase(t, DISABLED_END, CLOSE_ARC);
        break;
      case 4:
        inactiveFuelPhase(t, TRANS_SHIFT_END);
        break;
      case 5:
        activeArcWithFuelPhase(t, SHIFT1_END, FAR_ARC);
        break;
      case 6:
        inactiveFuelPhase(t, SHIFT2_END);
        break;
      case 7:
        activeArcWithFuelPhase(t, SHIFT3_END, CLOSE_ARC);
        break;
      case 8:
        endgamePhase(t);
        break;
      case 9:
        hangerPhase(t);
        break;
    }
  }

  /**
   * AUTO (3-23s, 20s): Sprint from Red wall toward hub area, shoot, grab fuel, sprint back, shoot
   * more. No arc drive in auto (no teleop command scheduler).
   */
  private void autoPhase(double t) {
    double at = t - PRE_MATCH_END;

    if (at < 2.0) {
      // Sprint from Red wall (Y=5.5 clears hub at Y=4.0)
      input.driveForward(0.95);
      input.strafe(0.15);
      startShooting();
    } else if (at < 5.0) {
      // Near hub area, shooting while drifting
      input.driveForward(0.1);
      input.strafe(-0.15 + 0.1 * Math.sin(at * 2.0));
    } else if (at < 8.0) {
      // Sprint to fuel zone, deploy intake
      input.driveForward(0.9);
      input.strafe(0.2);
      input.holdB(true);
    } else if (at < 10.0) {
      // Creep through fuel
      input.holdB(true);
      input.driveForward(0.2);
      input.strafe(0.1);
    } else if (at < 14.0) {
      // Sprint back toward hub
      input.holdB(false);
      input.driveForward(-0.9);
      input.strafe(-0.25);
    } else {
      // Park near hub, strafing
      input.driveForward(-0.05);
      input.strafe(-0.08 + 0.1 * Math.sin(at * 1.5));
    }
  }

  /**
   * ACTIVE ARC PHASE (10s transition shift): Pure arc orbit + shooting. No fuel runs,just orbit the
   * hub for the full duration.
   */
  private void activeArcPhase(double t, double phaseStart, double arcDist) {
    double pt = t - phaseStart;
    // Smooth oscillating strafe drives the arc sweep
    input.strafe(Math.sin(pt * 0.4) * 0.5);
  }

  /**
   * ACTIVE ARC WITH FUEL (25s shifts): Alternates between arc orbit (shooting) and fuel runs
   * (intake). More dynamic than pure arc.
   *
   * <p>0-8s: Arc orbit with shooting 8-18s: Fuel run cycle (stop arc, intake, return) 18-25s:
   * Resume arc orbit with shooting
   */
  private void activeArcWithFuelPhase(double t, double phaseStart, double arcDist) {
    double pt = t - phaseStart;

    if (pt < 8.0) {
      // Arc orbit phase,strafe oscillation
      input.strafe(Math.sin(pt * 0.4) * 0.5);
    } else if (pt < 18.0) {
      // Fuel run,temporarily stop arc drive for default drive
      double ft = pt - 8.0;
      if (ft < 0.1) {
        // Cancel arc drive to free swerve for default drive
        stopArcDrive();
      }
      if (ft < 3.0) {
        // Sprint to fuel (above hub Y=5.5 clears hub)
        input.driveForward(0.9);
        input.strafe(0.3);
        input.holdB(true);
      } else if (ft < 5.5) {
        // Creep through fuel zone
        input.holdB(true);
        input.driveForward(0.25);
        input.strafe(0.1);
      } else if (ft < 8.5) {
        // Sprint back to hub area
        input.holdB(false);
        input.driveForward(-0.9);
        input.strafe(-0.3);
      } else {
        // Settle near hub for arc resumption
        input.driveForward(-0.05);
        input.strafe(0.08 * Math.sin(ft * 2.0));
      }
    } else {
      // Resume arc orbit
      double at = pt - 18.0;
      if (at < 0.1) {
        startArcDrive(arcDist);
      }
      input.strafe(Math.sin(at * 0.45) * 0.55);
    }
  }

  /**
   * INACTIVE FUEL PHASE (25s): Hub INACTIVE,no shooting. 3 fuel collection runs across different
   * field zones.
   */
  private void inactiveFuelPhase(double t, double phaseStart) {
    double pt = t - phaseStart;

    if (pt < 8.0) {
      // Run 1: above hub (Y > 5)
      fuelRun(pt, 0.9, 0.8, -0.4);
    } else if (pt < 16.0) {
      // Run 2: upper far fuel
      fuelRun(pt - 8.0, 0.9, 0.9, -0.5);
    } else if (pt < 22.0) {
      // Run 3: below hub (Y < 3)
      fuelRun(pt - 16.0, 0.85, -0.9, 0.5);
    } else {
      // Reposition near hub for next active shift
      input.holdB(false);
      input.driveForward(-0.7);
      input.strafe(0.3);
    }
  }

  /**
   * ENDGAME (136-151s, 15s): Both hubs ACTIVE. Quick fuel run then arc orbit dump before hanger
   * climb.
   */
  private void endgamePhase(double t) {
    double pt = t - SHIFT4_END;

    if (pt < 6.0) {
      // Quick fuel grab
      stopArcDrive();
      if (pt < 2.5) {
        input.driveForward(0.9);
        input.strafe(0.3);
        input.holdB(true);
      } else if (pt < 4.0) {
        input.holdB(true);
        input.driveForward(0.2);
      } else {
        input.holdB(false);
        input.driveForward(-0.9);
        input.strafe(-0.3);
      }
    } else {
      // Final arc orbit dump
      double at = pt - 6.0;
      if (at < 0.1) {
        startArcDrive(FAR_ARC);
      }
      input.strafe(Math.sin(at * 0.5) * 0.6);
    }
  }

  /** HANGER CLIMB (151-166s, 15s): Sprint to Red Tower, deploy, climb. */
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
   * Fuel collection run (~8s) for INACTIVE shifts. Sprint to fuel, intake, sprint partway back. NO
   * SHOOTING.
   */
  private void fuelRun(double ct, double speed, double outBias, double returnBias) {
    if (ct < 3.0) {
      input.driveForward(speed);
      input.strafe(speed * 0.4 * outBias);
    } else if (ct < 5.5) {
      input.holdB(true);
      input.driveForward(0.25);
      input.strafe(0.1 * outBias);
    } else {
      input.holdB(false);
      input.driveForward(-speed * 0.8);
      input.strafe(speed * 0.3 * returnBias);
    }
  }

  private void startArcDrive(double distance) {
    stopArcDrive(); // cancel any existing arc first
    try {
      RobotContainer rc = RobotContainer.getInstance();
      if (rc == null) return;
      var swerve = rc.getSwerveSubsystem();
      arcDriveCmd =
          new HubArcDrive(
              swerve, SimDriveOverride::getX, RED_HUB_CENTER, distance, RED_SCORING_SIDE, 90.0);
      CommandScheduler.getInstance().schedule(arcDriveCmd);
    } catch (RuntimeException e) {
      SafeLog.put("Sim/FullShowcase/ArcError", e.getClass().getSimpleName());
    }
  }

  private void stopArcDrive() {
    if (arcDriveCmd != null) {
      try {
        CommandScheduler.getInstance().cancel(arcDriveCmd);
      } catch (RuntimeException ignored) {
      }
      arcDriveCmd = null;
    }
  }

  private void startShooting() {
    if (shooterCmd == null) {
      try {
        shooterCmd = new SpeedUpThenIndex();
        CommandScheduler.getInstance().schedule(shooterCmd);
      } catch (RuntimeException e) {
        shooterCmd = null;
      }
    }
  }

  private void stopShooting() {
    if (shooterCmd != null) {
      try {
        CommandScheduler.getInstance().cancel(shooterCmd);
      } catch (RuntimeException ignored) {
      }
      shooterCmd = null;
    }
  }

  private void stopAll() {
    stopArcDrive();
    stopShooting();
    input.driveForward(0);
    input.strafe(0);
    input.rotate(0);
  }
}
