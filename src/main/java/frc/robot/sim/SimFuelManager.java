package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ProjectileSimConstants;
import frc.robot.Constants.ShotCalculatorConstants;
import frc.robot.telemetry.SafeLog;
import frc.robot.util.TunableNumber;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** Wires SimDeviceManager shot detection to FuelPhysicsSim ball physics. */
public class SimFuelManager {
  private static final TunableNumber enabled = new TunableNumber("Sim/FuelSim/Enabled", 1);
  private static final TunableNumber subticks = new TunableNumber("Sim/FuelSim/Subticks", 5);
  private static final TunableNumber magnusToggle =
      new TunableNumber("Sim/FuelSim/MagnusEnabled", 1);
  private static final TunableNumber frictionToggle =
      new TunableNumber("Sim/FuelSim/FrictionEnabled", 1);
  private static final TunableNumber sleepingToggle =
      new TunableNumber("Sim/FuelSim/SleepingEnabled", 1);
  private static final TunableNumber ccdToggle = new TunableNumber("Sim/FuelSim/CCDEnabled", 1);

  private final FuelPhysicsSim sim;
  private boolean lastShotSignal = false;

  // Ball tracking: how many balls the robot is currently holding.
  // Only enforced when intake is registered (otherwise shots fire freely for backward compat).
  private int ballsHeld = 0;
  private boolean intakeRegistered = false;
  private static final int MAX_BALLS_HELD = 50;
  private static volatile boolean simBallPresent = true; // default true until intake is wired
  private boolean adaptiveShotSpeed = true; // true = always-score, false = real RPM physics

  // Actual ball trail: tracks the in-flight ball's XY after each shot for "predicted vs actual"
  private static final int MAX_TRAIL_POINTS = 100;
  private final Pose2d[] actualTrailBuffer = new Pose2d[MAX_TRAIL_POINTS];
  private int actualTrailCount = 0;
  private boolean trackingBall = false;
  private int prevTotalLaunched = 0;
  private int prevTotalScored = 0;
  private Pose2d lastRobotPose = null;
  private Pose2d[] actualBallTrail = new Pose2d[0];

  /** True if the robot has a ball in sim. Volatile so telemetry can read it safely. */
  public static boolean isSimBallPresent() {
    return simBallPresent;
  }

  /** When false, shots use real shooter RPM so wrong speed = miss. Default true (always-score). */
  public void setAdaptiveShotSpeed(boolean adaptive) {
    this.adaptiveShotSpeed = adaptive;
  }

  // Shot parameters from measured constants
  private static final double EXIT_HEIGHT_M = ProjectileSimConstants.EXIT_HEIGHT_M;
  private static final double FIXED_LAUNCH_ANGLE_DEG =
      ShotCalculatorConstants.FIXED_LAUNCH_ANGLE_DEG;
  private static final double WHEEL_DIAMETER_M = ProjectileSimConstants.WHEEL_DIAMETER_M;
  private static final double SLIP_FACTOR = 0.6; // matches ShotCalculator default
  private static final double DEFAULT_SPIN_RPM = 2000.0;

  // Hub geometry (must match FuelPhysicsSim for accurate scoring detection)
  private static final Translation2d BLUE_HUB = new Translation2d(4.5974, 4.035);
  private static final Translation2d RED_HUB = new Translation2d(11.938, 4.035);
  private static final double HUB_ENTRY_HEIGHT = 1.829; // m, 72 in
  private static final double FIELD_MID_X = 16.541 / 2.0;
  private static final double GRAVITY = 9.81;

  // FuelPhysicsSim applies aerodynamic drag (Cd=0.47) which slows the ball ~4-5%
  // over typical flight distances. This factor bumps the vacuum-trajectory exit
  // speed so the ball arrives with enough energy to clear the hub rim.
  private static final double DRAG_COMPENSATION = 1.05;

  public SimFuelManager(FuelPhysicsSim sim) {
    this.sim = sim;
  }

  public SimFuelManager(String tableKey) {
    this(new FuelPhysicsSim(tableKey));
  }

  /** Wire robot pose and speed suppliers so the sim can do bumper collisions. */
  public void configureRobot(
      double width,
      double length,
      double bumperHeight,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier) {
    sim.configureRobot(width, length, bumperHeight, poseSupplier, speedsSupplier);
  }

  /** Register a bounding box where balls get picked up when the intake is running. */
  public void addIntakeZone(
      double xMin, double xMax, double yMin, double yMax, BooleanSupplier active) {
    intakeRegistered = true;
    sim.addIntakeZone(
        xMin,
        xMax,
        yMin,
        yMax,
        active,
        () -> {
          if (ballsHeld < MAX_BALLS_HELD) {
            ballsHeld++;
          }
        });
  }

  /** Call each simulationPeriodic(). Runs shot detection, physics step, and logging. */
  public void update(boolean shotSignal, Pose2d robotPose, double robotYaw, double shooterRPM) {
    try {
      boolean isEnabled = enabled.get() >= 0.5;
      SafeLog.put("Sim/Fuel/Enabled", isEnabled);

      if (!isEnabled) return;

      // Update feature flags from TunableNumbers
      updateConfigFromTunables();

      // Detect rising edge of shot signal.
      // When intake is registered, require ballsHeld > 0 (realistic practice mode).
      // When no intake registered, fire freely (tests and scenarios).
      if (shotSignal && !lastShotSignal && shooterRPM > 100) {
        if (!intakeRegistered || ballsHeld > 0) {
          if (adaptiveShotSpeed) {
            launchAdaptive(robotPose, robotYaw);
          } else {
            launchAtRPM(robotPose, robotYaw, shooterRPM);
          }
          if (intakeRegistered && ballsHeld > 0) {
            ballsHeld--;
          }
        }
      }
      lastShotSignal = shotSignal;

      // Update static flag for ScoringTelemetry
      simBallPresent = !intakeRegistered || ballsHeld > 0;

      // Run physics
      long startNs = System.nanoTime();
      sim.tick();
      double elapsedMs = (System.nanoTime() - startNs) / 1_000_000.0;

      // Actual ball trail: track the highest in-flight ball each frame
      lastRobotPose = robotPose;
      updateActualBallTrail();

      // Log telemetry
      SafeLog.put("Sim/Fuel/BallsInFlight", sim.getBallsInFlight());
      SafeLog.put("Sim/Fuel/BallsOnGround", sim.getBallsOnGround());
      SafeLog.put("Sim/Fuel/TotalLaunched", sim.getTotalLaunched());
      SafeLog.put("Sim/Fuel/TotalScored", sim.getTotalScored());
      SafeLog.put("Sim/Fuel/TotalIntaked", sim.getTotalIntaked());
      SafeLog.put("Sim/Fuel/BallsHeld", ballsHeld);
      SafeLog.put("Sim/Fuel/LastLaunchSpeedMps", sim.getLastLaunchSpeed());
      SafeLog.put("Sim/Fuel/PhysicsStepMs", elapsedMs);
      SafeLog.put("Sim/Fuel/ActualBallTrail", actualBallTrail);

      // Height diagnostics: count balls in Z ranges to identify floating balls
      int above05 = 0, above1 = 0, above2 = 0;
      double maxZ = 0;
      for (var pos : sim.getBallPositions()) {
        double z = pos.getZ();
        if (z > maxZ) maxZ = z;
        if (z > 0.5) above05++;
        if (z > 1.0) above1++;
        if (z > 2.0) above2++;
      }
      SafeLog.put("Sim/Fuel/Debug/MaxZ", maxZ);
      SafeLog.put("Sim/Fuel/Debug/Above05m", above05);
      SafeLog.put("Sim/Fuel/Debug/Above1m", above1);
      SafeLog.put("Sim/Fuel/Debug/Above2m", above2);
    } catch (Throwable t) {
      // Never let fuel sim crash the robot sim loop
      SafeLog.put("Sim/Fuel/Error", t.getClass().getSimpleName());
    }
  }

  /**
   * Track the highest in-flight ball's XY position each frame after a new shot launches. When the
   * ball scores (gets removed by BallPhysicsSim), extend the trail to the hub center so you see the
   * full path. When the ball misses and lands, freeze the trail as-is.
   */
  private void updateActualBallTrail() {
    int totalLaunched = sim.getTotalLaunched();
    int totalScored = sim.getTotalScored();

    // New shot fired: reset trail and start tracking
    if (totalLaunched > prevTotalLaunched) {
      trackingBall = true;
      actualTrailCount = 0;
    }
    prevTotalLaunched = totalLaunched;

    // Ball scored: extend trail to hub center and stop tracking
    if (trackingBall
        && totalScored > prevTotalScored
        && actualTrailCount > 0
        && actualTrailCount < MAX_TRAIL_POINTS) {
      Translation2d hub = (lastRobotPose != null) ? getNearestHub(lastRobotPose) : BLUE_HUB;
      double dx = hub.getX() - actualTrailBuffer[actualTrailCount - 1].getX();
      double dy = hub.getY() - actualTrailBuffer[actualTrailCount - 1].getY();
      actualTrailBuffer[actualTrailCount++] =
          new Pose2d(hub.getX(), hub.getY(), new Rotation2d(Math.atan2(dy, dx)));
      actualBallTrail = new Pose2d[actualTrailCount];
      System.arraycopy(actualTrailBuffer, 0, actualBallTrail, 0, actualTrailCount);
      trackingBall = false;
    }
    prevTotalScored = totalScored;

    if (!trackingBall) return;

    // Find the highest-Z ball (most likely the one we just shot)
    List<Translation3d> positions = sim.getBallPositions();
    Translation3d highest = null;
    double maxZ = -1;
    for (Translation3d pos : positions) {
      if (pos.getZ() > maxZ) {
        maxZ = pos.getZ();
        highest = pos;
      }
    }

    // Record its XY if still in flight (Z > 0.3m, above ground resting height)
    if (highest != null && maxZ > 0.3 && actualTrailCount < MAX_TRAIL_POINTS) {
      double dx = 0, dy = 0;
      if (actualTrailCount > 0) {
        dx = highest.getX() - actualTrailBuffer[actualTrailCount - 1].getX();
        dy = highest.getY() - actualTrailBuffer[actualTrailCount - 1].getY();
      }
      actualTrailBuffer[actualTrailCount++] =
          new Pose2d(highest.getX(), highest.getY(), new Rotation2d(Math.atan2(dy, dx)));

      // Build output array
      actualBallTrail = new Pose2d[actualTrailCount];
      System.arraycopy(actualTrailBuffer, 0, actualBallTrail, 0, actualTrailCount);
    } else if (maxZ <= 0.3) {
      // Ball missed and landed on ground, stop tracking
      trackingBall = false;
    }
  }

  /**
   * Compute vacuum-trajectory exit speed to lob a ball from distance d at our fixed angle into the
   * hub entry. Adds drag compensation for FuelPhysicsSim air resistance.
   *
   * <p>From projectile kinematics: v² = g·d² / (2·cos²(θ)·(z₀ + d·tan(θ) - h))
   *
   * @return required exit speed in m/s, or -1 if the shot geometry is impossible
   */
  private double computeExitSpeedForDistance(double horizontalDist) {
    double theta = Math.toRadians(FIXED_LAUNCH_ANGLE_DEG);
    double cosTheta = Math.cos(theta);
    double tanTheta = Math.tan(theta);
    double denom =
        2.0 * cosTheta * cosTheta * (EXIT_HEIGHT_M + horizontalDist * tanTheta - HUB_ENTRY_HEIGHT);
    if (denom <= 0) return -1;
    return DRAG_COMPENSATION * Math.sqrt(GRAVITY * horizontalDist * horizontalDist / denom);
  }

  /** Pick the hub on the robot's half of the field. */
  private Translation2d getNearestHub(Pose2d robotPose) {
    return (robotPose.getX() > FIELD_MID_X) ? RED_HUB : BLUE_HUB;
  }

  /**
   * Launch a ball with distance-adaptive exit speed. Computes the exact speed needed to reach the
   * nearest hub from the robot's current position, so shots score from any distance (not just one
   * tuned distance).
   */
  private void launchAdaptive(Pose2d robotPose, double robotYaw) {
    Translation3d launchPos = new Translation3d(robotPose.getX(), robotPose.getY(), EXIT_HEIGHT_M);

    Translation2d hub = getNearestHub(robotPose);
    double dx = hub.getX() - robotPose.getX();
    double dy = hub.getY() - robotPose.getY();
    double horizontalDist = Math.sqrt(dx * dx + dy * dy);

    double exitSpeed = computeExitSpeedForDistance(horizontalDist);
    if (exitSpeed < 0) exitSpeed = 7.0; // fallback for impossible geometry

    double launchAngleRad = Math.toRadians(FIXED_LAUNCH_ANGLE_DEG);
    double vHorizontal = exitSpeed * Math.cos(launchAngleRad);
    double vVertical = exitSpeed * Math.sin(launchAngleRad);

    double vx = vHorizontal * Math.cos(robotYaw);
    double vy = vHorizontal * Math.sin(robotYaw);

    Translation3d launchVel = new Translation3d(vx, vy, vVertical);
    sim.launchBall(launchPos, launchVel, DEFAULT_SPIN_RPM);
  }

  /** Launch a ball using raw RPM-to-speed conversion. For testing specific RPM values. */
  private void launchAtRPM(Pose2d robotPose, double robotYaw, double shooterRPM) {
    Translation3d launchPos = new Translation3d(robotPose.getX(), robotPose.getY(), EXIT_HEIGHT_M);

    double launchAngleRad = Math.toRadians(FIXED_LAUNCH_ANGLE_DEG);
    double exitSpeed = SLIP_FACTOR * shooterRPM * Math.PI * WHEEL_DIAMETER_M / 60.0;
    double vHorizontal = exitSpeed * Math.cos(launchAngleRad);
    double vVertical = exitSpeed * Math.sin(launchAngleRad);

    double vx = vHorizontal * Math.cos(robotYaw);
    double vy = vHorizontal * Math.sin(robotYaw);

    Translation3d launchVel = new Translation3d(vx, vy, vVertical);
    sim.launchBall(launchPos, launchVel, DEFAULT_SPIN_RPM);
  }

  private void updateConfigFromTunables() {
    FuelPhysicsSim.PhysicsConfig cfg = sim.getConfig();
    cfg.subticks = Math.max(1, (int) subticks.get());
    cfg.magnusEnabled = magnusToggle.get() >= 0.5;
    cfg.frictionEnabled = frictionToggle.get() >= 0.5;
    cfg.sleepingEnabled = sleepingToggle.get() >= 0.5;
    cfg.ccdEnabled = ccdToggle.get() >= 0.5;
  }

  /**
   * Force-launch a ball at a specific RPM, bypassing normal shot detection. Used by test scenarios
   * to sweep RPM ranges.
   */
  public void forceShot(Pose2d robotPose, double robotYaw, double overrideRPM) {
    launchAtRPM(robotPose, robotYaw, overrideRPM);
  }

  /**
   * Force-launch with distance-adaptive exit speed. Computes the right speed to score from any
   * distance. Used by distance sweep test scenarios.
   */
  public void forceShotAdaptive(Pose2d robotPose, double robotYaw) {
    launchAdaptive(robotPose, robotYaw);
  }

  public FuelPhysicsSim getSim() {
    return sim;
  }

  public void placeFieldBalls() {
    sim.placeFieldBalls();
  }

  public void enable() {
    sim.enable();
  }

  public void disable() {
    sim.disable();
  }

  public void clearBalls() {
    sim.clearBalls();
  }
}
