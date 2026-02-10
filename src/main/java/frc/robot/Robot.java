// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.telemetry.CycleTracker;
import frc.robot.telemetry.TelemetryManager;
import frc.robot.util.AlertManager;
import frc.robot.util.DiagnosticContext;
import frc.robot.util.ElasticUtil;
import frc.robot.util.EventMarker;
import frc.robot.util.LoggedTracer;
import frc.robot.util.PostMatchSummary;
import frc.robot.util.PreMatchDiagnostics;
import frc.robot.util.PredictiveAlerts;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private static Robot instance;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  // Diagnostics
  private boolean hasRunDiagnostics = false;

  // Logging status
  private boolean loggingAvailable = false;

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  private void installExceptionHandler() {
    Thread.setDefaultUncaughtExceptionHandler(
        (thread, exception) -> {
          try {
            DiagnosticContext.captureException(thread, exception);
          } catch (Throwable t) {
          }

          if (exception instanceof RuntimeException) {
            throw (RuntimeException) exception;
          } else if (exception instanceof Error) {
            throw (Error) exception;
          } else {
            throw new RuntimeException(exception);
          }
        });
  }

  // ==================== LOGGING CONFIGURATION ====================

  /** Configure AdvantageKit logging. Failures are swallowed so the robot keeps running. */
  private void configureLogging() {
    try {
      // Record build metadata for traceability in logs
      Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
      Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
      Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
      Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
      Logger.recordMetadata("GitDirty", BuildConstants.DIRTY == 1 ? "true" : "false");

      if (isReal()) {
        // USB logging - optional, fail gracefully if USB not mounted
        try {
          Logger.addDataReceiver(new WPILOGWriter());
        } catch (Throwable t) {
          DriverStation.reportWarning("USB logging unavailable: " + t.getMessage(), false);
        }

        // NT logging - also optional
        try {
          Logger.addDataReceiver(new NT4Publisher());
        } catch (Throwable t) {
          DriverStation.reportWarning("NT logging unavailable: " + t.getMessage(), false);
        }
      } else {
        // Simulation - less critical but still wrap
        try {
          Logger.addDataReceiver(new WPILOGWriter("logs"));
          Logger.addDataReceiver(new NT4Publisher());
        } catch (Throwable t) {
          // Simulation logging failed - continue anyway
        }
      }

      Logger.start();
      loggingAvailable = true;
    } catch (Throwable t) {
      // Entire logging system failed - robot still runs
      DriverStation.reportError("Logging system failed to initialize: " + t.getMessage(), false);
      loggingAvailable = false;
    }
  }

  /** Safe logging helper that cannot throw */
  private void safeLog(String key, boolean value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
      // Ignore - logging failure shouldn't crash robot
    }
  }

  /** Safe logging helper that cannot throw */
  private void safeLog(String key, String value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
      // Ignore - logging failure shouldn't crash robot
    }
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    installExceptionHandler();
    configureLogging();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    m_robotContainer = RobotContainer.getInstance();

    // Create a timer to disable motor brake a few seconds after disable. This will
    // let the robot stop immediately when disabled, but then also let it be pushed more
    disabledTimer = new Timer();

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Send test notification to Elastic Dashboard (protected)
    try {
      ElasticUtil.sendInfo("Robot", "Code initialized successfully");
    } catch (Throwable t) {
      // Dashboard notification failed - not critical
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    LoggedTracer.reset();

    // CommandScheduler is CRITICAL - robot must drive
    // But even this shouldn't crash the entire robot
    try {
      CommandScheduler.getInstance().run();
    } catch (Throwable t) {
      safeLog("Health/CrashBarrier/CommandScheduler", true);
      safeLog("Health/CrashBarrier/LastError", t.getClass().getSimpleName());
    }
    LoggedTracer.record("CommandsMs");

    // Telemetry is IMPORTANT but not critical - robot can play without it
    try {
      TelemetryManager.getInstance().updateAll();
    } catch (Throwable t) {
      safeLog("Health/CrashBarrier/Telemetry", true);
      safeLog("Health/CrashBarrier/LastError", t.getClass().getSimpleName());
    }
    LoggedTracer.record("TelemetryMs");

    // Alerts are OPTIONAL - robot definitely plays without them
    try {
      AlertManager.getInstance().checkAll();
      AlertManager.getInstance().checkLoopTime(TelemetryManager.getInstance().getLoopTimeMs());
      AlertManager.getInstance().logActiveAlerts();
    } catch (Throwable t) {
      safeLog("Health/CrashBarrier/Alerts", true);
      safeLog("Health/CrashBarrier/LastError", t.getClass().getSimpleName());
    }

    // Predictive alerts are OPTIONAL
    try {
      PredictiveAlerts.getInstance().update();
      PredictiveAlerts.getInstance().log();
    } catch (Throwable t) {
      safeLog("Health/CrashBarrier/Predictive", true);
      safeLog("Health/CrashBarrier/LastError", t.getClass().getSimpleName());
    }

    // Post-match tracking is OPTIONAL
    try {
      if (DriverStation.isEnabled()) {
        PostMatchSummary.getInstance()
            .updateTracking(TelemetryManager.getInstance().getLoopTimeMs());
      }
    } catch (Throwable t) {
      safeLog("Health/CrashBarrier/PostMatch", true);
      safeLog("Health/CrashBarrier/LastError", t.getClass().getSimpleName());
    }

    LoggedTracer.record("AlertsMs");
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();

    try {
      EventMarker.modeChange("DISABLED");
    } catch (Throwable t) {
      // Event marker failure not critical
    }

    // Generate post-match summary if we were tracking
    try {
      PostMatchSummary.getInstance().generateSummary();
    } catch (Throwable t) {
      safeLog("Health/CrashBarrier/PostMatchSummary", true);
    }

    // Reset diagnostics flag so they run once in disabledPeriodic
    hasRunDiagnostics = false;
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }

    // Run safe diagnostics once after 0.5s delay (let readings stabilize)
    if (!hasRunDiagnostics && disabledTimer.hasElapsed(0.5)) {
      try {
        PreMatchDiagnostics.getInstance().runSafeChecks();
      } catch (Throwable t) {
        safeLog("Health/CrashBarrier/Diagnostics", true);
        DriverStation.reportWarning("Pre-match diagnostics failed: " + t.getMessage(), false);
      }
      hasRunDiagnostics = true;
    }

    // Manual safe trigger from dashboard (sensor-only, works while disabled)
    try {
      if (PreMatchDiagnostics.getInstance().checkAndClearSafeTrigger()
          && !PreMatchDiagnostics.getInstance().isRunning()) {
        PreMatchDiagnostics.getInstance().runSafeChecks();
      }
    } catch (Throwable t) {
      safeLog("Health/CrashBarrier/DiagnosticsTrigger", true);
    }

    // Full trigger pressed while disabled - warn user
    try {
      if (PreMatchDiagnostics.getInstance().checkAndClearFullTrigger()) {
        safeLog("Diagnostics/FullCheckBlocked", true);
        // Notification sent by PreMatchDiagnostics.runFullChecks() when it rejects
      }
    } catch (Throwable t) {
      // Ignore
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Reset event markers and cycle tracker for new match
    EventMarker.reset();
    CycleTracker.getInstance().reset();
    EventMarker.modeChange("AUTONOMOUS");

    // Start tracking for post-match summary
    PostMatchSummary.getInstance().startTracking();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
      String autoName = m_autonomousCommand.getName();
      EventMarker.autoStart(autoName);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    CommandScheduler.getInstance().cancelAll();
    EventMarker.modeChange("TELEOP");

    // Start tracking for post-match summary (if not already from auto)
    PostMatchSummary.getInstance().startTracking();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    EventMarker.modeChange("TEST");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // Full diagnostic trigger (actuator tests) - only works in test mode
    try {
      if (PreMatchDiagnostics.getInstance().checkAndClearFullTrigger()
          && !PreMatchDiagnostics.getInstance().isRunning()) {
        PreMatchDiagnostics.getInstance().runFullChecks();
      }
      // Safe checks can also run in test mode
      if (PreMatchDiagnostics.getInstance().checkAndClearSafeTrigger()
          && !PreMatchDiagnostics.getInstance().isRunning()) {
        PreMatchDiagnostics.getInstance().runSafeChecks();
      }
    } catch (Throwable t) {
      safeLog("Health/CrashBarrier/DiagnosticsTest", true);
    }
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
