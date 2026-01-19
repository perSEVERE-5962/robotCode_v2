// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.Cameras;

// AdvantageKit imports for logging
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {

  private static Robot instance;
  private Command m_autonomousCommand;
  private Cameras cameras;
  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  // Loop timing tracking
  private double lastLoopTimestamp = 0;
  private int loopOverrunCount = 0;

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  // ==================== LOGGING HELPER METHODS ====================

  /**
   * Returns the current robot mode as a string.
   */
  private String getCurrentMode() {
    if (isDisabled()) return "Disabled";
    if (isAutonomous()) return "Autonomous";
    if (isTeleop()) return "Teleop";
    if (isTest()) return "Test";
    return "Unknown";
  }

  /**
   * Returns the alliance color as a string.
   */
  private String getAllianceString() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get().name();
    }
    return "Unknown";
  }

  /**
   * Logs match state data every cycle.
   * Signals: Match/Time, Mode, Enabled, FMSAttached, MatchNumber, EventName, Alliance, StationNumber
   */
  private void logMatchData() {
    Logger.recordOutput("Match/Time", DriverStation.getMatchTime());
    Logger.recordOutput("Match/Mode", getCurrentMode());
    Logger.recordOutput("Match/Enabled", isEnabled());
    Logger.recordOutput("Match/FMSAttached", DriverStation.isFMSAttached());
    Logger.recordOutput("Match/MatchNumber", DriverStation.getMatchNumber());
    Logger.recordOutput("Match/EventName", DriverStation.getEventName());
    Logger.recordOutput("Match/Alliance", getAllianceString());
    Logger.recordOutput("Match/StationNumber", DriverStation.getLocation().orElse(0));
  }

  // Track active commands via callbacks
  private final java.util.Set<String> activeCommands = new java.util.LinkedHashSet<>();

  /**
   * Sets up command tracking callbacks.
   * Called once during robotInit.
   */
  private void setupCommandLogging() {
    CommandScheduler.getInstance().onCommandInitialize(command -> {
      activeCommands.add(command.getName());
    });
    CommandScheduler.getInstance().onCommandFinish(command -> {
      activeCommands.remove(command.getName());
    });
    CommandScheduler.getInstance().onCommandInterrupt(command -> {
      activeCommands.remove(command.getName());
    });
  }

  /**
   * Logs active command data every cycle.
   * Signals: Commands/ActiveList, Commands/ActiveCount
   */
  private void logCommands() {
    String list = activeCommands.isEmpty() ? "none" : String.join(", ", activeCommands);
    Logger.recordOutput("Commands/ActiveList", list);
    Logger.recordOutput("Commands/ActiveCount", activeCommands.size());
  }

  /**
   * Logs system health data every cycle.
   * Signals: BatteryVoltage, CAN stats, RIO voltages/temp, BrownedOut, RSLState
   */
  private void logSystemHealth() {
    // Battery voltage
    Logger.recordOutput("SystemHealth/BatteryVoltage", RobotController.getBatteryVoltage());

    // CAN bus statistics
    var canStatus = RobotController.getCANStatus();
    Logger.recordOutput("SystemHealth/CANUtilization", canStatus.percentBusUtilization);
    Logger.recordOutput("SystemHealth/CANTxErrors", canStatus.transmitErrorCount);
    Logger.recordOutput("SystemHealth/CANRxErrors", canStatus.receiveErrorCount);
    Logger.recordOutput("SystemHealth/CANBusOff", canStatus.busOffCount);

    // roboRIO health
    Logger.recordOutput("SystemHealth/RioCPUTemp", RobotController.getCPUTemp());
    Logger.recordOutput("SystemHealth/Rio3V3Rail", RobotController.getVoltage3V3());
    Logger.recordOutput("SystemHealth/Rio5VRail", RobotController.getVoltage5V());
    Logger.recordOutput("SystemHealth/Rio6VRail", RobotController.getVoltage6V());

    // System state
    Logger.recordOutput("SystemHealth/BrownedOut", RobotController.isBrownedOut());
    Logger.recordOutput("SystemHealth/RSLState", RobotController.getRSLState());
  }

  /**
   * Logs loop timing data every cycle.
   * Tracks loop time in milliseconds and counts overruns (> 20ms).
   * Signals: SystemHealth/LoopTimeMs, SystemHealth/LoopOverruns
   */
  private void logLoopTiming() {
    double currentTimestamp = Timer.getFPGATimestamp();

    // Calculate loop time (skip first cycle where lastLoopTimestamp is 0)
    if (lastLoopTimestamp > 0) {
      double loopTimeMs = (currentTimestamp - lastLoopTimestamp) * 1000.0;

      // Check for overrun (> 20ms for 50Hz loop)
      if (loopTimeMs > 20.0) {
        loopOverrunCount++;
      }

      Logger.recordOutput("SystemHealth/LoopTimeMs", loopTimeMs);
    }

    Logger.recordOutput("SystemHealth/LoopOverruns", loopOverrunCount);
    lastLoopTimestamp = currentTimestamp;
  }

  // ==================== LOGGING CONFIGURATION ====================

  /**
   * Configures AdvantageKit logging.
   * - Records build metadata for traceability
   * - Sets up file logging (WPILOGWriter) to USB drive
   * - Sets up NetworkTables publishing (NT4Publisher) for live dashboard
   */
  private void configureLogging() {
    // Record build metadata for traceability in logs
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("GitDirty", BuildConstants.DIRTY == 1 ? "true" : "false");

    // Set up data receivers (where logged data goes)
    if (isReal()) {
      // Running on real robot: log to USB drive and publish to NetworkTables
      Logger.addDataReceiver(new WPILOGWriter()); // Log to USB stick at /U/logs
      Logger.addDataReceiver(new NT4Publisher()); // Publish to NetworkTables
    } else {
      // Running in simulation: log to local file and publish to NetworkTables
      Logger.addDataReceiver(new WPILOGWriter("logs")); // Log to logs/ folder
      Logger.addDataReceiver(new NT4Publisher());
    }

    // Start the logger - must be called after all configuration
    Logger.start();
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Configure AdvantageKit logging FIRST before anything else
    configureLogging();

    // Set up command tracking callbacks
    setupCommandLogging();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = RobotContainer.getInstance();

    // Create a timer to disable motor brake a few seconds after disable. This will
    // let the robot stop
    // immediately when disabled, but then also let it be pushed more
    disabledTimer = new Timer();

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Log loop timing at the start to measure time since last cycle
    logLoopTiming();

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Log match state data every cycle
    logMatchData();

    // Log system health data every cycle
    logSystemHealth();

    // Log active commands every cycle
    logCommands();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit() {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic() {
  }
}
