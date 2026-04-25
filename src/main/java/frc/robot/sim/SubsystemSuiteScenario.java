package frc.robot.sim;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.util.SafeLog;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * Field-aware scenario: starts in center zone (between hubs), navigates around obstacles, drives to
 * fuel, picks up, drives to scoring position, shoots.
 *
 * <p>Field layout (2026 REBUILT): - Center zone: x=5.19 to x=11.34 (between hub ramps) - Red Hub:
 * (11.94, 4.03) with ramp collider y=1.28 to y=6.79 - Fuel patch: (7.36-9.18, 1.72-6.25), center
 * ~(8.27, 4.0) - Robot starts at (9, 5, 180°),open area near fuel
 *
 * <p>100s total, 14 phases (12 original + copilot exercise + copilot shoot).
 *
 * <p>Launch: ./gradlew simulateJava -DsimScenario=SubsystemSuite
 */
class SubsystemSuiteScenario implements SimScenario {
  private double startTime;
  private boolean finished = false;
  private final SimInputPlayback input = new SimInputPlayback();
  private int lastPhase = -1;
  private boolean xPressed = false;

  @Override
  public String getName() {
    return "SubsystemSuite";
  }

  @Override
  public void init() {
    startTime = Timer.getFPGATimestamp();
    finished = false;
    lastPhase = -1;
    xPressed = false;
    input.releaseAll();
    SimulatedBattery.disableBatterySim(); // prevent MapleSim from overwriting our voltage ramp
    RoboRioSim.setVInVoltage(12.5);
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
  }

  @Override
  public void execute(double timestampSec) {
    double t = timestampSec - startTime;
    int phase = getPhase(t);

    if (phase != lastPhase) {
      SafeLog.put("Sim/Suite/Phase", phase);
      SafeLog.put("Sim/Suite/PhaseName", phaseName(phase));
      lastPhase = phase;
      onPhaseEnter(phase);
    }

    onPhaseUpdate(phase, t);

    SafeLog.put("Sim/Suite/ElapsedSec", t);
    SafeLog.put("Sim/Suite/Progress", Math.min(100, (t / 100.0) * 100));

    if (t >= 100.0) {
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
    if (t < 2) return 0; // Disabled baseline
    if (t < 10) return 1; // Drive to fuel area
    if (t < 18) return 2; // Shooter spin-up
    if (t < 25) return 3; // Indexer run
    if (t < 33) return 4; // Intake deploy + run
    if (t < 40) return 5; // Drive to scoring position + shoot
    if (t < 50) return 6; // SpeedUpThenIndex rapid fire
    if (t < 58) return 7; // Drive circle + shoot
    if (t < 66) return 8; // Hanger deploy/climb
    if (t < 76) return 9; // All subsystems combined
    if (t < 82) return 10; // Copilot: DeployIntake + AgitateAndIndex
    if (t < 88) return 11; // Copilot: SpeedUpThenIndex via right trigger
    if (t < 94) return 12; // Drive back to fuel + intake
    return 13; // Cooldown + disable
  }

  private String phaseName(int phase) {
    switch (phase) {
      case 0:
        return "DisabledBaseline";
      case 1:
        return "DriveToFuel";
      case 2:
        return "ShooterSpinUp";
      case 3:
        return "IndexerRun";
      case 4:
        return "IntakeDeploy";
      case 5:
        return "DriveToHubAndShoot";
      case 6:
        return "RapidFire";
      case 7:
        return "DriveCircleShoot";
      case 8:
        return "HangerCycle";
      case 9:
        return "AllSubsystems";
      case 10:
        return "CopilotIntakeAgitate";
      case 11:
        return "CopilotShoot";
      case 12:
        return "DriveBackIntake";
      case 13:
        return "Cooldown";
      default:
        return "Unknown";
    }
  }

  private void onPhaseEnter(int phase) {
    // Release drive inputs between phases
    input.driveForward(0);
    input.strafe(0);
    input.rotate(0);

    // Release all action buttons,each phase explicitly enables what it needs
    stopShooting();
    input.holdA(false);
    input.holdB(false);
    input.holdY(false);
    input.holdRightBumper(false);

    switch (phase) {
      case 0: // Disabled baseline,release everything
        input.releaseAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        break;

      case 1: // Enable teleop
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setMatchTime(135.0);
        DriverStationSim.notifyNewData();
        break;

      case 13: // Cooldown, release everything
        input.releaseAll();
        break;
    }
  }

  private void onPhaseUpdate(int phase, double t) {
    switch (phase) {
      case 1:
        { // Drive to fuel: robot at (9,5,180°), drive toward fuel at (8.5,3.5)
          double driveT = t - 2.0;
          if (driveT < 3.0) {
            input.driveForward(0.5);
            input.strafe(-0.3);
          } else if (driveT < 5.0) {
            input.strafe(-0.4);
            input.driveForward(0.2);
          } else {
            input.driveForward(-0.3);
            input.rotate(0.5);
          }
          break;
        }

      case 2: // Shooter spin-up: A only (MoveShooter, no X conflict)
        input.holdA(true);
        break;

      case 3: // Indexer: Y only (no shooter)
        input.holdY(true);
        break;

      case 4:
        { // Intake deploy: B for deploy+intake, then release to retract
          double intakeT = t - 25.0;
          if (intakeT < 5.0) {
            input.holdB(true);
          } else {
            input.holdB(false);
          }
          break;
        }

      case 5:
        { // Drive toward Red Hub scoring position + shoot
          double dt = t - 33.0;
          if (dt < 3.0) {
            input.driveForward(-0.6);
            input.strafe(0.2);
            // Use X for shooting (SpeedUpThenIndex handles shooter+indexer)
            // NO A,avoids command conflict
            startShooting();
          } else {
            // Keep X held,SpeedUpThenIndex running
            startShooting();
          }
          break;
        }

      case 6:
        { // SpeedUpThenIndex rapid fire: X only, no A
          double fireT = t - 40.0;
          double cycle = fireT % 3.0;
          if (cycle < 1.0) {
            // Brief pause between shots,release X
            stopShooting();
          } else {
            // SpeedUpThenIndex handles spin-up + indexing
            startShooting();
          }
          break;
        }

      case 7:
        { // Drive in circle near hub while shooting
          double circT = t - 50.0;
          input.driveForward(0.3 * Math.sin(circT * 0.8));
          input.strafe(0.3 * Math.cos(circT * 0.8));
          input.rotate(0.2);
          // Shoot every 2.5s,X only, no A
          if (circT % 2.5 > 0.8) {
            startShooting();
          } else {
            stopShooting();
          }
          break;
        }

      case 8:
        { // Hanger deploy and climb (no shooter)
          double hangerT = t - 58.0;
          if (hangerT < 4.0) {
            input.holdRightBumper(true);
          } else {
            input.holdRightBumper(false);
          }
          break;
        }

      case 9:
        { // All subsystems: drive + shoot + intake + hanger
          double allT = t - 66.0;
          input.driveForward(0.4 * Math.sin(allT * 0.6));
          input.strafe(0.3 * Math.cos(allT * 0.4));
          // Cycle intake every 3s (B button, no subsystem conflict with X)
          input.holdB(allT % 3.0 < 1.5);
          // Cycle hanger every 4s
          input.holdRightBumper(allT % 4.0 < 2.0);
          // Shoot every 3s,X only (handles shooter+indexer together)
          // No A or Y,they'd conflict with SpeedUpThenIndex
          if (allT % 3.0 > 1.0) {
            startShooting();
          } else {
            stopShooting();
          }
          break;
        }

      case 10:
        { // Copilot: DeployIntake(A) + AgitateAndIndex(B) + PivotIntake(bumpers)
          double copT = t - 76.0;
          if (copT < 2.5) {
            // Deploy intake via copilot A (DeployIntake + HoldAndIntake)
            input.copilotHoldA(true);
          } else if (copT < 4.5) {
            input.copilotHoldA(false);
            // AgitateAndIndex via copilot B
            input.copilotHoldB(true);
          } else {
            input.copilotHoldB(false);
            // PivotIntake down then up via copilot bumpers
            input.copilotHoldRightBumper(copT < 5.5);
            input.copilotHoldLeftBumper(copT >= 5.5);
          }
          break;
        }

      case 11:
        { // Copilot: SpeedUpThenIndex via right trigger
          double copT = t - 82.0;
          if (copT < 4.0) {
            // Hold right trigger for SpeedUpThenIndex
            input.copilotSetRightTrigger(1.0);
          } else {
            input.copilotSetRightTrigger(0);
          }
          break;
        }

      case 12:
        { // Drive back to fuel + intake
          double driveT = t - 88.0;
          if (driveT < 4.0) {
            input.driveForward(0.5);
            input.holdB(true);
          } else {
            input.strafe(0.4 * Math.sin(driveT * 1.0));
            input.holdB(true);
          }
          break;
        }

      case 13:
        { // Cooldown, slow drive, fade out
          double coolT = t - 94.0;
          double fade = Math.max(0, 1.0 - coolT / 5.0);
          input.driveForward(0.2 * fade);
          input.rotate(0.1 * fade);
          if (coolT > 5.0) {
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
          }
          break;
        }
    }
  }

  /**
   * Start shooting: X only (SpeedUpThenIndex handles both shooter+indexer). NEVER combine with A
   * (MoveShooter),they fight over the Shooter subsystem.
   */
  private void startShooting() {
    input.holdA(false); // release A to avoid conflict
    input.holdY(false); // release Y,SpeedUpThenIndex owns indexer too
    if (!xPressed) {
      input.tapX();
      xPressed = true;
    }
  }

  /** Stop shooting: release X, SpeedUpThenIndex ends. */
  private void stopShooting() {
    if (xPressed) {
      input.releaseX();
      xPressed = false;
    }
  }
}
