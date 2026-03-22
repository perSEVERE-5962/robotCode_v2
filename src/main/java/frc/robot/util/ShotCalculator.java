package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.HubScoringConstants;
import frc.robot.Constants.ShotCalculatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.telemetry.SafeLog;

/**
 * Ballistic fire control for competition. Newton TOF solver, LUT lookup, band multipliers, and SOTM
 * heading compensation. calculate() runs once per robotPeriodic(), commands read the cache.
 */
public class ShotCalculator {

  public record LaunchParameters(
      double rpm,
      double hoodAngleDeg,
      double timeOfFlightSec,
      Rotation2d driveAngle,
      double driveAngularVelocityRadPerSec,
      boolean isValid,
      double confidence,
      boolean passing) {
    public static final LaunchParameters INVALID =
        new LaunchParameters(0, 0, 0, new Rotation2d(), 0, false, 0, false);
  }

  private static ShotCalculator instance;

  public static ShotCalculator getInstance() {
    if (instance == null) {
      instance = new ShotCalculator();
    }
    return instance;
  }

  // Tunable parameters
  private final TunableNumber kMinSOTMSpeed =
      new TunableNumber("ShotCalc/minSOTMSpeed", ShotCalculatorConstants.MIN_SOTM_SPEED);
  private final TunableNumber kPhaseDelayMs =
      new TunableNumber("ShotCalc/phaseDelayMs", ShotCalculatorConstants.PHASE_DELAY_MS);
  private final TunableNumber kMechLatencyMs =
      new TunableNumber("ShotCalc/mechLatencyMs", ShotCalculatorConstants.MECH_LATENCY_MS);
  private final TunableNumber kMaxIterations =
      new TunableNumber("ShotCalc/maxIterations", ShotCalculatorConstants.MAX_ITERATIONS);
  private final TunableNumber kConvergenceTolerance =
      new TunableNumber(
          "ShotCalc/convergenceTolerance", ShotCalculatorConstants.CONVERGENCE_TOLERANCE);
  private final TunableNumber kMaxSOTMSpeed =
      new TunableNumber("ShotCalc/maxSOTMSpeed", ShotCalculatorConstants.MAX_SOTM_SPEED);
  private final TunableNumber kSOTMDragCoeff =
      new TunableNumber("ShotCalc/sotmDragCoeff", ShotCalculatorConstants.SOTM_DRAG_COEFF);

  // Per-distance band RPM multipliers for field-day calibration
  private final TunableNumber kRpmMultShort = new TunableNumber("ShotCalc/RPMMultShort", 1.0);
  private final TunableNumber kRpmMultMedium = new TunableNumber("ShotCalc/RPMMultMedium", 1.0);
  private final TunableNumber kRpmMultLong = new TunableNumber("ShotCalc/RPMMultLong", 1.0);

  // nonzero = bypass LUT and use this RPM for every shot
  private final TunableNumber kRpmOverride = new TunableNumber("ShotCalc/RPMOverride", 0);

  private final ShotLUT baseLUT = new ShotLUT();

  // Solver state (reused across cycles)
  private double previousTOF = -1;
  private int iterationsUsed = 0;
  private boolean warmStartUsed = false;
  private boolean velocityFiltered = false;
  private boolean behindHub = false;
  private boolean speedCapped = false;
  private double solvedDistance = 0;

  // Previous robot-relative velocity for acceleration estimation
  private double prevRobotVx = 0;
  private double prevRobotVy = 0;
  private double prevRobotOmega = 0;
  private double prevTimestamp = -1;

  private double polarSpeedLimitMps = 0;
  private LaunchParameters cachedParameters = LaunchParameters.INVALID;
  private SwerveSubsystem swerve;

  private ShotCalculator() {
    initializeBaselineLUT();
  }

  /** Starting LUT from physics sim. Needs real-robot calibration. */
  private void initializeBaselineLUT() {
    double angle = ShotCalculatorConstants.FIXED_LAUNCH_ANGLE_DEG;
    baseLUT.put(0.50, 2800.0, angle, 0.42);
    baseLUT.put(1.00, 3200.0, angle, 0.62);
    baseLUT.put(1.20, 3400.0, angle, 0.70);
    baseLUT.put(1.50, 3550.0, angle, 0.80);
    baseLUT.put(2.00, 3750.0, angle, 0.93);
    baseLUT.put(2.50, 3950.0, angle, 1.07);
    baseLUT.put(3.00, 4150.0, angle, 1.22);
    baseLUT.put(3.50, 4350.0, angle, 1.40);
    baseLUT.put(4.00, 4550.0, angle, 1.59);
    baseLUT.put(4.50, 4750.0, angle, 1.80);
    baseLUT.put(5.00, 4950.0, angle, 2.03);
  }

  /** Set swerve reference. Called from RobotContainer during init. */
  public void setSwerve(SwerveSubsystem swerve) {
    this.swerve = swerve;
  }

  // LUT access with per-distance band multiplier
  double effectiveRPM(double distance) {
    return baseLUT.getRPM(distance) * getDistanceBandMultiplier(distance);
  }

  private double getDistanceBandMultiplier(double distance) {
    if (distance < ShotCalculatorConstants.RPM_BAND_SHORT_END) return kRpmMultShort.get();
    if (distance < ShotCalculatorConstants.RPM_BAND_MEDIUM_END) return kRpmMultMedium.get();
    return kRpmMultLong.get();
  }

  double effectiveTOF(double distance) {
    return baseLUT.getTOF(distance);
  }

  double effectiveAngle(double distance) {
    return baseLUT.getAngle(distance);
  }

  /** TOF adjusted for inherited velocity drag decay. Reduces to raw tof when drag coeff ~ 0. */
  private double dragCompensatedTOF(double tof) {
    double c = kSOTMDragCoeff.get();
    if (c < 1e-6) return tof;
    return (1.0 - Math.exp(-c * tof)) / c;
  }

  private static final double DERIV_H = 0.01;

  double tofMapDerivative(double d) {
    double tHigh = effectiveTOF(d + DERIV_H);
    double tLow = effectiveTOF(d - DERIV_H);
    return (tHigh - tLow) / (2.0 * DERIV_H);
  }

  /** Run once per robotPeriodic(). Caches the result for commands to read. */
  public LaunchParameters calculate() {
    if (swerve == null) {
      cachedParameters = LaunchParameters.INVALID;
      return cachedParameters;
    }

    try {
      cachedParameters = computeSolution();
    } catch (Throwable t) {
      cachedParameters = LaunchParameters.INVALID;
    }

    logTelemetry();
    return cachedParameters;
  }

  private LaunchParameters computeSolution() {
    Pose2d rawPose = swerve.getPose();
    ChassisSpeeds robotVel = swerve.getRobotVelocity();

    if (rawPose == null || robotVel == null) {
      return LaunchParameters.INVALID;
    }
    double poseX = rawPose.getX();
    double poseY = rawPose.getY();
    if (Double.isNaN(poseX)
        || Double.isNaN(poseY)
        || Double.isInfinite(poseX)
        || Double.isInfinite(poseY)) {
      return LaunchParameters.INVALID;
    }

    // Second-order pose prediction
    double now = Timer.getFPGATimestamp();
    double cycleDt = (prevTimestamp > 0) ? MathUtil.clamp(now - prevTimestamp, 0.005, 0.1) : 0.02;
    prevTimestamp = now;
    double dt = kPhaseDelayMs.get() / 1000.0;
    double ax = (robotVel.vxMetersPerSecond - prevRobotVx) / cycleDt;
    double ay = (robotVel.vyMetersPerSecond - prevRobotVy) / cycleDt;
    double aOmega = (robotVel.omegaRadiansPerSecond - prevRobotOmega) / cycleDt;
    Pose2d compensatedPose =
        rawPose.exp(
            new Twist2d(
                robotVel.vxMetersPerSecond * dt + 0.5 * ax * dt * dt,
                robotVel.vyMetersPerSecond * dt + 0.5 * ay * dt * dt,
                robotVel.omegaRadiansPerSecond * dt + 0.5 * aOmega * dt * dt));
    prevRobotVx = robotVel.vxMetersPerSecond;
    prevRobotVy = robotVel.vyMetersPerSecond;
    prevRobotOmega = robotVel.omegaRadiansPerSecond;

    double robotX = compensatedPose.getX();
    double robotY = compensatedPose.getY();
    double heading = compensatedPose.getRotation().getRadians();

    Translation2d hubCenter = getHubCenter();
    double hubX = hubCenter.getX();
    double hubY = hubCenter.getY();

    // Behind-hub detection
    Translation2d hubForward = getHubForward();
    double dot = (hubX - robotX) * hubForward.getX() + (hubY - robotY) * hubForward.getY();
    behindHub = dot < 0;
    if (behindHub) {
      return LaunchParameters.INVALID;
    }

    // Transform robot center to launcher position
    double cosH = Math.cos(heading);
    double sinH = Math.sin(heading);
    double launcherX =
        robotX
            + ShotCalculatorConstants.LAUNCHER_OFFSET_X * cosH
            - ShotCalculatorConstants.LAUNCHER_OFFSET_Y * sinH;
    double launcherY =
        robotY
            + ShotCalculatorConstants.LAUNCHER_OFFSET_X * sinH
            + ShotCalculatorConstants.LAUNCHER_OFFSET_Y * cosH;

    // Robot field velocity from measured robot-relative velocity
    double robotFieldVx = robotVel.vxMetersPerSecond * cosH - robotVel.vyMetersPerSecond * sinH;
    double robotFieldVy = robotVel.vxMetersPerSecond * sinH + robotVel.vyMetersPerSecond * cosH;

    // Launcher velocity: v_robot_field + omega x r_launcher
    double launcherFieldOffX =
        ShotCalculatorConstants.LAUNCHER_OFFSET_X * cosH
            - ShotCalculatorConstants.LAUNCHER_OFFSET_Y * sinH;
    double launcherFieldOffY =
        ShotCalculatorConstants.LAUNCHER_OFFSET_X * sinH
            + ShotCalculatorConstants.LAUNCHER_OFFSET_Y * cosH;
    double omega = robotVel.omegaRadiansPerSecond;
    double vx = robotFieldVx + (-launcherFieldOffY) * omega;
    double vy = robotFieldVy + launcherFieldOffX * omega;

    // Displacement from launcher to hub
    double rx = hubX - launcherX;
    double ry = hubY - launcherY;
    double distance = Math.hypot(rx, ry);
    solvedDistance = distance;

    if (distance < ShotCalculatorConstants.MIN_SCORING_DISTANCE
        || distance > ShotCalculatorConstants.MAX_SCORING_DISTANCE) {
      return LaunchParameters.INVALID;
    }

    double robotSpeed = Math.hypot(vx, vy);

    // Speed cap
    speedCapped = robotSpeed > kMaxSOTMSpeed.get();
    if (speedCapped) {
      return LaunchParameters.INVALID;
    }

    velocityFiltered = robotSpeed < kMinSOTMSpeed.get();

    double solvedTOF;
    double projDist;

    if (velocityFiltered) {
      // Static shot: no velocity compensation
      solvedTOF = effectiveTOF(distance);
      projDist = distance;
      iterationsUsed = 0;
      warmStartUsed = false;
    } else {
      // Newton TOF solver
      int maxIter = (int) kMaxIterations.get();
      double convTol = kConvergenceTolerance.get();

      double tof;
      if (previousTOF > 0) {
        tof = previousTOF;
        warmStartUsed = true;
      } else {
        tof = effectiveTOF(distance);
        warmStartUsed = false;
      }

      projDist = distance;
      iterationsUsed = 0;

      for (int i = 0; i < maxIter; i++) {
        double prevTOF = tof;

        double driftTOF = dragCompensatedTOF(tof);
        double prx = rx - vx * driftTOF;
        double pry = ry - vy * driftTOF;
        projDist = Math.hypot(prx, pry);

        if (projDist < 0.01) {
          tof = effectiveTOF(distance);
          iterationsUsed = maxIter + 1;
          break;
        }

        double lookupTOF = effectiveTOF(projDist);

        // Analytical derivative
        double dragDeriv = Math.exp(-kSOTMDragCoeff.get() * tof);
        double dPrime = -dragDeriv * (prx * vx + pry * vy) / projDist;
        double gPrime = tofMapDerivative(projDist);
        double f = lookupTOF - tof;
        double fPrime = gPrime * dPrime - 1.0;

        if (Math.abs(fPrime) > 0.01) {
          tof = tof - f / fPrime;
        } else {
          tof = lookupTOF;
        }

        tof = MathUtil.clamp(tof, ShotCalculatorConstants.TOF_MIN, ShotCalculatorConstants.TOF_MAX);
        iterationsUsed = i + 1;

        if (Math.abs(tof - prevTOF) < convTol) {
          break;
        }
      }

      // Divergence guard
      if (tof > ShotCalculatorConstants.TOF_MAX || tof < 0.0 || Double.isNaN(tof)) {
        tof = effectiveTOF(distance);
        iterationsUsed = (int) kMaxIterations.get() + 1;
      }

      solvedTOF = tof;
    }

    previousTOF = solvedTOF;
    double effectiveTOFValue = solvedTOF + kMechLatencyMs.get() / 1000.0;
    double overrideRPM = kRpmOverride.get();
    double effectiveRPMValue = (overrideRPM > 0) ? overrideRPM : effectiveRPM(projDist);

    // Drive angle: aim at velocity-compensated target position
    double compTargetX;
    double compTargetY;
    if (velocityFiltered) {
      compTargetX = hubX;
      compTargetY = hubY;
    } else {
      double headingDriftTOF = dragCompensatedTOF(solvedTOF);
      compTargetX = hubX - vx * headingDriftTOF;
      compTargetY = hubY - vy * headingDriftTOF;
    }

    double aimX = compTargetX - robotX;
    double aimY = compTargetY - robotY;
    Rotation2d driveAngle =
        new Rotation2d(aimX, aimY)
            .plus(new Rotation2d(ShotCalculatorConstants.SHOOTER_ANGLE_OFFSET_RAD));

    // Angular velocity feedforward
    double driveAngularVelocity = 0;
    if (!velocityFiltered && distance > 0.1) {
      double tangentialVel = (ry * vx - rx * vy) / distance;
      driveAngularVelocity = tangentialVel / distance;
    }

    // Simple polar speed limit
    polarSpeedLimitMps = computePolarSpeedLimit(distance, 2.0);

    // Stripped confidence: valid = 100, invalid = 0
    double confidence = 100.0;
    boolean passing = false;
    boolean isValid = true;

    double hoodAngle = effectiveAngle(projDist);

    return new LaunchParameters(
        effectiveRPMValue,
        hoodAngle,
        effectiveTOFValue,
        driveAngle,
        driveAngularVelocity,
        isValid,
        confidence,
        passing);
  }

  /** Max driver speed before the aim can't keep up with the angular rate. */
  public static double computePolarSpeedLimit(double distanceM, double maxAngularRateRadPerSec) {
    if (distanceM < 0.5) return 0.5;
    return maxAngularRateRadPerSec * distanceM;
  }

  public Translation2d getHubCenter() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return HubScoringConstants.RED_HUB_CENTER;
    }
    return HubScoringConstants.BLUE_HUB_CENTER;
  }

  /** Hub forward vector points from hub toward the scoring side. */
  public Translation2d getHubForward() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return new Translation2d(-1, 0);
    }
    return new Translation2d(1, 0);
  }

  private void logTelemetry() {
    SafeLog.put("ShotCalc/EffectiveRPM", cachedParameters.rpm());
    SafeLog.put("ShotCalc/HoodAngleDeg", cachedParameters.hoodAngleDeg());
    SafeLog.put("ShotCalc/EffectiveTOF", cachedParameters.timeOfFlightSec());
    SafeLog.put("ShotCalc/Distance", solvedDistance);
    SafeLog.put(
        "ShotCalc/DriveAngleDeg", Math.toDegrees(cachedParameters.driveAngle().getRadians()));
    SafeLog.put("ShotCalc/IsValid", cachedParameters.isValid());
    SafeLog.put("ShotCalc/ConvergenceIterations", iterationsUsed);
    SafeLog.put("ShotCalc/WarmStartUsed", warmStartUsed);
    SafeLog.put("ShotCalc/VelocityFiltered", velocityFiltered);
    SafeLog.put("ShotCalc/BehindHub", behindHub);
    SafeLog.put("ShotCalc/SpeedCapped", speedCapped);
    SafeLog.put("ShotCalc/HubCenterX", getHubCenter().getX());
    SafeLog.put("ShotCalc/HubCenterY", getHubCenter().getY());
    SafeLog.put("ShotCalc/PolarSpeedLimitMps", polarSpeedLimitMps);
    SafeLog.put("ShotCalc/RPMOverrideActive", kRpmOverride.get() > 0);
  }

  public LaunchParameters getParameters() {
    return cachedParameters;
  }

  public double getSolvedDistance() {
    return solvedDistance;
  }

  public double getPolarSpeedLimitMps() {
    return polarSpeedLimitMps;
  }

  /** Returns the dashboard RPM override, or 0 if not set. */
  public double getRpmOverride() {
    return kRpmOverride.get();
  }
}
