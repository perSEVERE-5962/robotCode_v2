package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Logs Pose3d[] trajectories for AdvantageScope 3D field visualization. When a shot fires, computes
 * a parabolic arc from robot to hub.
 */
public class ShotVisualizerTelemetry implements SubsystemTelemetry {
  // Red Hub field position (2026 REBUILT)
  private static final double HUB_X = 11.90;
  private static final double HUB_Y = 4.0;
  private static final double HUB_Z = 2.0;
  private static final double SHOOTER_EXIT_Z = 0.8;
  private static final double ARC_PEAK_HEIGHT = 1.5;
  private static final int ARC_POINTS = 20;
  private static final double DISPLAY_DURATION_SEC = 0.6;

  private final ShooterTelemetry shooterTelemetry;
  private SwerveSubsystem swerveSubsystem;

  private int prevTotalShots = 0;
  private Pose3d[] activeTrajectory = new Pose3d[0];
  private double trajectoryStartTime = 0;

  public ShotVisualizerTelemetry(ShooterTelemetry shooterTelemetry) {
    this.shooterTelemetry = shooterTelemetry;
  }

  public void setSwerveSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
  }

  @Override
  public void update() {
    try {
      if (shooterTelemetry == null || swerveSubsystem == null) return;

      double now = Timer.getFPGATimestamp();
      int totalShots = shooterTelemetry.getTotalShots();

      // New shot detected
      if (totalShots > prevTotalShots) {
        Pose2d robotPose = swerveSubsystem.getPose();
        if (robotPose != null) {
          activeTrajectory = computeArc(robotPose);
          trajectoryStartTime = now;
        }
      }
      prevTotalShots = totalShots;

      // Clear after display duration
      if (activeTrajectory.length > 0 && (now - trajectoryStartTime) > DISPLAY_DURATION_SEC) {
        activeTrajectory = new Pose3d[0];
      }
    } catch (Throwable t) {
      activeTrajectory = new Pose3d[0];
    }
  }

  private Pose3d[] computeArc(Pose2d robotPose) {
    double startX = robotPose.getX();
    double startY = robotPose.getY();

    Pose3d[] points = new Pose3d[ARC_POINTS];
    for (int i = 0; i < ARC_POINTS; i++) {
      double t = (double) i / (ARC_POINTS - 1);
      double x = startX + (HUB_X - startX) * t;
      double y = startY + (HUB_Y - startY) * t;
      double zLinear = SHOOTER_EXIT_Z + (HUB_Z - SHOOTER_EXIT_Z) * t;
      double arcBulge = ARC_PEAK_HEIGHT * 4.0 * t * (1.0 - t);
      points[i] = new Pose3d(x, y, zLinear + arcBulge, new Rotation3d());
    }
    return points;
  }

  @Override
  public void log() {
    SafeLog.put("Scoring/ShotTrajectory", activeTrajectory);
  }

  @Override
  public String getName() {
    return "ShotVisualizer";
  }
}
