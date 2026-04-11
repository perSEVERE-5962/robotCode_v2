// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

public class HitResistantFollowPath extends Command {

  public static double RECOVERY_THRESHOLD = 0.50;

  public static double REENTRY_THRESHOLD = 0.30;

  //  Seconds after the first execute() call during which recovery cannot
  //  trigger. Set at the end of initialize().

  private static final double RECOVERY_SUPPRESSION_TIME = 1.0;

  // Seconds after exiting reentry during which recovery cannot trigger again.

  private static final double POST_REENTRY_COOLDOWN = 1.0;

  // How far ahead (trajectory-seconds) to place the intercept point.
  public static double LOOKAHEAD_TIME = 1.0;

  // Ghost-bot time scale while in RECOVERY (0–1).
  public static double GHOST_BOT_MIN_SCALE = 0.25;

  // Scale units per second at which ghost-bot ramps back to 1.0 in reentry.
  public static double RAMP_RATE = 2.0;

  // Recovery path constraints. */
  public static double RECOVERY_MAX_VEL = 3.5;
  public static double RECOVERY_MAX_ACCEL = 3.0;
  public static double RECOVERY_MAX_OMEGA = Units.degreesToRadians(540);
  public static double RECOVERY_MAX_ALPHA = Units.degreesToRadians(720);

  public enum FollowState {
    NOMINAL, // standard following, ghost scale = 1.0
    RECOVERY, // displaced; on-the-fly path active, ghost scale = MIN
    REENTRY // near intercept; ghost scale ramping back to 1.0
  }

  private record ScheduledEvent(double triggerTimeSeconds, Command command, String label) {}

  private final SwerveSubsystem swerve;
  private final PathPlannerPath originalPath;
  private final RobotConfig robotConfig;
  private final boolean resetOdometryOnStart;

  private final PPHolonomicDriveController mainController;

  private PathPlannerTrajectory trajectory;
  private PathPlannerPath activePath;

  private double virtualTime = 0.0;
  private double ghostBotScale = 1.0;

  private FollowState followState = FollowState.NOMINAL;
  private FollowState lastLoggedState = FollowState.NOMINAL;

  private Command recoverySubCommand = null;
  private boolean recoveryActive = false;

  private double prevTimestamp;
  private double startTimestamp;
  private double reentryExitTimestamp = -1.0;
  private double errorExceededTimestamp = -1.0; // Debounce timer

  private final List<ScheduledEvent> eventSchedule = new ArrayList<>();
  private final Set<Integer> firedEventIndices = new HashSet<>();
  private final List<Command> runningEventCommands = new ArrayList<>();

  public HitResistantFollowPath(
      SwerveSubsystem swerve,
      PathPlannerPath path,
      RobotConfig robotConfig,
      boolean resetOdometryOnStart) {
    this.swerve = swerve;
    this.originalPath = path;
    this.robotConfig = robotConfig;
    this.resetOdometryOnStart = resetOdometryOnStart;

    this.mainController =
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0));

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // Reset state
    virtualTime = 0.0;
    ghostBotScale = 1.0;
    followState = FollowState.NOMINAL;
    lastLoggedState = FollowState.NOMINAL;
    reentryExitTimestamp = -1.0;
    errorExceededTimestamp = -1.0;

    firedEventIndices.clear();
    runningEventCommands.clear();
    eventSchedule.clear();
    recoverySubCommand = null;
    recoveryActive = false;

    // Build trajectory
    activePath = shouldFlipPath() ? originalPath.flipPath() : originalPath;

    ChassisSpeeds currentSpeeds = swerve.getRobotVelocity();
    Rotation2d currentHeading = swerve.getHeading();
    trajectory = activePath.generateTrajectory(currentSpeeds, currentHeading, robotConfig);

    if (resetOdometryOnStart) {
      swerve.resetOdometry(trajectory.getInitialState().pose);
    }

    mainController.reset(swerve.getPose(), currentSpeeds);
    buildEventSchedule();

    prevTimestamp = Timer.getFPGATimestamp();
    startTimestamp = Timer.getFPGATimestamp();

    SmartDashboard.putString("HitResistant/State", "NOMINAL");
    SmartDashboard.putNumber("HitResistant/TrajDuration", trajectory.getTotalTimeSeconds());
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    double realDt = Math.min(now - prevTimestamp, 0.05);
    prevTimestamp = now;

    Pose2d currentPose = swerve.getPose();

    // Advance virtual clock
    virtualTime = Math.min(virtualTime + realDt * ghostBotScale, trajectory.getTotalTimeSeconds());

    PathPlannerTrajectoryState ghostState = trajectory.sample(virtualTime);

    // cross track error
    Translation2d errorVector =
        ghostState.pose.getTranslation().minus(currentPose.getTranslation());
    Translation2d pathRelativeError = errorVector.rotateBy(ghostState.heading.unaryMinus());
    double poseError = Math.abs(pathRelativeError.getY());

    switch (followState) {
      case NOMINAL -> executeNominal(currentPose, ghostState, poseError, now);
      case RECOVERY -> executeRecovery(currentPose, poseError);
      case REENTRY -> executeReentry(currentPose, ghostState, poseError, realDt);
    }

    processEventCommands();
    tickRunningEventCommands();

    // Telemetry
    SmartDashboard.putString("HitResistant/State", followState.name());
    SmartDashboard.putNumber("HitResistant/VirtualTime", virtualTime);
    SmartDashboard.putNumber("HitResistant/GhostScale", ghostBotScale);
    SmartDashboard.putNumber("HitResistant/PoseError", poseError);
    SmartDashboard.putNumber(
        "HitResistant/TrajRemaining", trajectory.getTotalTimeSeconds() - virtualTime);

    if (followState != lastLoggedState) {
      System.out.println(
          "[HitResistant] "
              + lastLoggedState
              + " -> "
              + followState
              + " | error="
              + String.format("%.3fm", poseError)
              + " | t="
              + String.format("%.2fs", virtualTime));
      lastLoggedState = followState;
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds());
    if (recoveryActive && recoverySubCommand != null) {
      recoverySubCommand.end(true);
    }
    for (Command c : runningEventCommands) {
      c.end(true);
    }
    runningEventCommands.clear();
  }

  @Override
  public boolean isFinished() {
    return virtualTime >= trajectory.getTotalTimeSeconds() && !recoveryActive;
  }

  private void executeNominal(
      Pose2d currentPose, PathPlannerTrajectoryState ghostState, double poseError, double now) {

    boolean startupSuppressed = (now - startTimestamp) < RECOVERY_SUPPRESSION_TIME;
    boolean cooldownActive =
        reentryExitTimestamp > 0 && (now - reentryExitTimestamp) < POST_REENTRY_COOLDOWN;

    if (!startupSuppressed && !cooldownActive && poseError > RECOVERY_THRESHOLD) {
      if (errorExceededTimestamp < 0) {
        errorExceededTimestamp = now;
      } else if (now - errorExceededTimestamp > 0.2) {
        enterRecovery(currentPose);
        errorExceededTimestamp = -1.0;
        return;
      }
    } else {
      errorExceededTimestamp = -1.0;
    }

    swerve.drive(mainController.calculateRobotRelativeSpeeds(currentPose, ghostState));
  }

  private void enterRecovery(Pose2d currentPose) {
    followState = FollowState.RECOVERY;
    ghostBotScale = GHOST_BOT_MIN_SCALE;

    double interceptTime = findBestInterceptTime();
    PathPlannerTrajectoryState interceptState = trajectory.sample(interceptTime);

    recoverySubCommand = buildRecoverySubCommand(currentPose, interceptState);
    recoverySubCommand.initialize();
    recoveryActive = true;

    SmartDashboard.putNumber("HitResistant/InterceptTime", interceptTime);
  }

  private void executeRecovery(Pose2d currentPose, double poseError) {
    if (!recoveryActive || recoverySubCommand == null) {
      followState = FollowState.REENTRY;
      return;
    }

    recoverySubCommand.execute();

    boolean subCommandDone = recoverySubCommand.isFinished();
    boolean errorDroppedEarly = poseError < RECOVERY_THRESHOLD * 0.65;

    if (subCommandDone || errorDroppedEarly) {
      recoverySubCommand.end(false);
      recoveryActive = false;

      virtualTime = findClosestTrajectoryTime(currentPose);

      followState = FollowState.REENTRY;
      mainController.reset(currentPose, swerve.getRobotVelocity());
    }
  }

  private void executeReentry(
      Pose2d currentPose, PathPlannerTrajectoryState ghostState, double poseError, double realDt) {
    ghostBotScale = Math.min(1.0, ghostBotScale + RAMP_RATE * realDt);

    swerve.drive(mainController.calculateRobotRelativeSpeeds(currentPose, ghostState));

    // Allow clean handoff without demanding perfect sub-inch accuracy
    if (ghostBotScale >= 1.0 && poseError < REENTRY_THRESHOLD) {
      reentryExitTimestamp = Timer.getFPGATimestamp();
      followState = FollowState.NOMINAL;
      mainController.reset(currentPose, swerve.getRobotVelocity());
    }
  }

  private double findBestInterceptTime() {
    return Math.min(virtualTime + LOOKAHEAD_TIME, trajectory.getTotalTimeSeconds());
  }

  //  Finds the trajectory time whose pose is spatially closest to currentPose.
  //  searches only forward so it doesn't snap backwards and cause the robot to twirl.

  private double findClosestTrajectoryTime(Pose2d currentPose) {
    double totalTime = trajectory.getTotalTimeSeconds();
    double bestTime = virtualTime;
    double bestDist = Double.MAX_VALUE;
    double maxSearchTime = Math.min(virtualTime + 2.0, totalTime);

    for (double t = virtualTime; t <= maxSearchTime; t += 0.05) {
      double dist =
          trajectory.sample(t).pose.getTranslation().getDistance(currentPose.getTranslation());
      if (dist < bestDist) {
        bestDist = dist;
        bestTime = t;
      }
    }
    return bestTime;
  }

  private Command buildRecoverySubCommand(
      Pose2d startPose, PathPlannerTrajectoryState interceptState) {
    PathConstraints constraints =
        new PathConstraints(
            RECOVERY_MAX_VEL, RECOVERY_MAX_ACCEL,
            RECOVERY_MAX_OMEGA, RECOVERY_MAX_ALPHA);

    Rotation2d departureHeading =
        new Rotation2d(
            interceptState.pose.getX() - startPose.getX(),
            interceptState.pose.getY() - startPose.getY());

    Pose2d wpStart = new Pose2d(startPose.getTranslation(), departureHeading);
    Pose2d wpEnd = new Pose2d(interceptState.pose.getTranslation(), interceptState.heading);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(wpStart, wpEnd);

    GoalEndState goalEndState =
        new GoalEndState(interceptState.linearVelocity, interceptState.pose.getRotation());

    PathPlannerPath recoveryPath = new PathPlannerPath(waypoints, constraints, null, goalEndState);
    recoveryPath.preventFlipping = true;

    ChassisSpeeds robotSpeed = swerve.getRobotVelocity();
    PathPlannerTrajectory recoveryTraj =
        recoveryPath.generateTrajectory(robotSpeed, swerve.getHeading(), robotConfig);

    PPHolonomicDriveController recoveryCtrl =
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0));
    Timer recoveryTimer = new Timer();

    return new Command() {
      @Override
      public void initialize() {
        recoveryTimer.reset();
        recoveryTimer.start();
        recoveryCtrl.reset(swerve.getPose(), swerve.getRobotVelocity());
      }

      @Override
      public void execute() {
        double t = Math.min(recoveryTimer.get(), recoveryTraj.getTotalTimeSeconds());
        ChassisSpeeds speeds =
            recoveryCtrl.calculateRobotRelativeSpeeds(swerve.getPose(), recoveryTraj.sample(t));
        swerve.drive(speeds);
      }

      @Override
      public boolean isFinished() {
        // 0.5s to allow the robot to physically catch up
        return recoveryTimer.hasElapsed(recoveryTraj.getTotalTimeSeconds() + 0.5);
      }

      @Override
      public void end(boolean interrupted) {
        recoveryTimer.stop();
      }
    };
  }

  private void buildEventSchedule() {
    List<EventMarker> markers = activePath.getEventMarkers();
    if (markers.isEmpty()) return;

    int numPoints = activePath.numPoints();
    double totalTime = trajectory.getTotalTimeSeconds();

    for (EventMarker marker : markers) {
      double relPos = marker.position();
      double fraction = (numPoints > 1) ? relPos / (numPoints - 1.0) : 0.0;
      fraction = Math.max(0.0, Math.min(1.0, fraction));

      double triggerTime = arcLengthFractionToTime(fraction, totalTime);
      Command cmd = marker.command();
      if (cmd != null) {
        eventSchedule.add(
            new ScheduledEvent(triggerTime, cmd, "event@t=" + String.format("%.2f", triggerTime)));
      }
    }

    eventSchedule.sort(Comparator.comparingDouble(ScheduledEvent::triggerTimeSeconds));
    SmartDashboard.putNumber("HitResistant/EventCount", eventSchedule.size());
  }

  private double arcLengthFractionToTime(double fraction, double totalTime) {
    int samples = Math.max(2, (int) (totalTime * 50));
    double dt = totalTime / (samples - 1);

    double[] cumArc = new double[samples];
    cumArc[0] = 0.0;
    PathPlannerTrajectoryState prev = trajectory.sample(0.0);

    for (int i = 1; i < samples; i++) {
      PathPlannerTrajectoryState curr = trajectory.sample(i * dt);
      cumArc[i] =
          cumArc[i - 1] + prev.pose.getTranslation().getDistance(curr.pose.getTranslation());
      prev = curr;
    }

    double targetLen = fraction * cumArc[samples - 1];
    int lo = 0, hi = samples - 1;
    while (hi - lo > 1) {
      int mid = (lo + hi) / 2;
      if (cumArc[mid] < targetLen) lo = mid;
      else hi = mid;
    }

    double span = cumArc[hi] - cumArc[lo];
    double t = (span < 1e-9) ? lo * dt : (lo + (targetLen - cumArc[lo]) / span) * dt;

    return Math.min(t, totalTime);
  }

  private void processEventCommands() {
    for (int i = 0; i < eventSchedule.size(); i++) {
      if (firedEventIndices.contains(i)) continue;
      ScheduledEvent event = eventSchedule.get(i);
      if (virtualTime >= event.triggerTimeSeconds()) {
        Command cmd = event.command();
        cmd.initialize();
        runningEventCommands.add(cmd);
        firedEventIndices.add(i);
        SmartDashboard.putString("HitResistant/LastFiredEvent", event.label());
      }
    }
  }

  private void tickRunningEventCommands() {
    Iterator<Command> it = runningEventCommands.iterator();
    while (it.hasNext()) {
      Command c = it.next();
      c.execute();
      if (c.isFinished()) {
        c.end(false);
        it.remove();
      }
    }
  }

  private boolean shouldFlipPath() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  public FollowState getFollowState() {
    return followState;
  }

  public double getGhostBotScale() {
    return ghostBotScale;
  }

  public double getVirtualTime() {
    return virtualTime;
  }
}
