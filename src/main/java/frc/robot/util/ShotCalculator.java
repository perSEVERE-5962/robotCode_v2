package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.HubScoringConstants;
import frc.robot.Constants.ShotCalculatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.telemetry.SafeLog;

/**
 * Ballistic fire control. Newton-method TOF solver with analytical derivative for shoot-on-the-move
 * heading compensation. Produces a cached LaunchParameters record every cycle.
 *
 * <p>Call calculate() once per robotPeriodic() before CommandScheduler.run(). Commands read
 * getParameters() for the cached result.
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

  // Tunable parameters (all serve as kill switches via dashboard)
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

  // per-distance band RPM multipliers for field-day calibration.
  // all default to 1.0 (no change). the drive team bumps these on the
  // dashboard if shots at a particular range are consistently off.
  private final TunableNumber kRpmMultShort = new TunableNumber("ShotCalc/RPMMultShort", 1.0);
  private final TunableNumber kRpmMultMedium = new TunableNumber("ShotCalc/RPMMultMedium", 1.0);
  private final TunableNumber kRpmMultLong = new TunableNumber("ShotCalc/RPMMultLong", 1.0);

  private final TunableNumber kRpmOverride = new TunableNumber("ShotCalc/RPMOverride", 0.0);

  private final ShotLUT baseLUT = new ShotLUT();
  private final InterpolatingDoubleTreeMap correctionRpmMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap correctionTofMap = new InterpolatingDoubleTreeMap();

  // copilot D-pad RPM trim, applied on top of LUT + per-distance corrections
  private double rpmOffset = 0;

  // Solver state (reused across cycles to avoid allocation)
  private double previousTOF = -1;
  private double previousSpeed = 0;
  private int iterationsUsed = 0;
  private boolean warmStartUsed = false;
  private boolean velocityFiltered = false;
  private boolean behindHub = false;
  private boolean speedCapped = false;
  private double solvedDistance = 0;

  // previous robot-relative velocity for acceleration estimation
  private double prevRobotVx = 0;
  private double prevRobotVy = 0;
  private double prevRobotOmega = 0;
  private double prevTimestamp = 0;

  // shot diagnostics state, captured per cycle for visualization
  private double diagRobotX = 0, diagRobotY = 0;
  private double diagHubX = 0, diagHubY = 0;
  private double diagVx = 0, diagVy = 0;
  private double diagOmega = 0;
  private double diagLauncherOffX = 0, diagLauncherOffY = 0;
  private double diagCompTargetX = 0, diagCompTargetY = 0;
  // solver convergence trail
  private static final int MAX_CONVERGENCE_STEPS = 25;
  private final double[] convergenceTOF = new double[MAX_CONVERGENCE_STEPS];
  private final double[] convergenceDist = new double[MAX_CONVERGENCE_STEPS];
  private final double[] convergenceResidual = new double[MAX_CONVERGENCE_STEPS];
  private int convergenceCount = 0;
  private final double[] diagTofTrail = new double[MAX_CONVERGENCE_STEPS];
  private final double[] diagDistTrail = new double[MAX_CONVERGENCE_STEPS];
  private final double[] diagResidualTrail = new double[MAX_CONVERGENCE_STEPS];
  // drag-compensated vs raw TOF comparison
  private double diagRawTOF = 0;
  private double diagDragCompTOF = 0;
  private double diagSolvedTOF = 0;

  // SOTM pipeline enhancements
  private final TunableNumber kMaxPolarAngularRate =
      new TunableNumber(
          "ShotCalc/maxPolarAngularRate",
          ShotCalculatorConstants.MAX_POLAR_ANGULAR_RATE_RAD_PER_SEC);
  private final TunableNumber kCrossTrackTolDeg =
      new TunableNumber(
          "ShotCalc/crossTrackToleranceDeg", ShotCalculatorConstants.CROSS_TRACK_TOLERANCE_DEG);
  private final TunableNumber kAlongTrackTolDeg =
      new TunableNumber(
          "ShotCalc/alongTrackToleranceDeg", ShotCalculatorConstants.ALONG_TRACK_TOLERANCE_DEG);

  // copilot aim bias: the copilot can nudge the aim point left/right with their
  // stick to correct for LUT inaccuracy at specific distances without a code change.
  private final TunableNumber kMaxAimBiasDeg = new TunableNumber("ShotCalc/maxAimBiasDeg", 5.0);
  private java.util.function.DoubleSupplier aimBiasSupplier = () -> 0;
  private double appliedAimBiasDeg = 0;

  private double polarSpeedLimitMps = 0;
  private double velocityToHubAngleDeg = 0;
  private double horizontalCompMps = 0;
  private double verticalCompRPM = 0;
  private double crossTrackErrorDeg = 0;
  private double alongTrackErrorDeg = 0;
  private double targetAngularRate = 0;

  private LaunchParameters cachedParameters = LaunchParameters.INVALID;

  private SwerveSubsystem swerve;

  private ShotCalculator() {
    initializeBaselineLUT();
  }

  /**
   * 81-point dense LUT: sim-generated at 0.05m steps (slip=0.5, Cd=0.47, 60 deg, exit=0.5309m),
   * with real robot RPM values overriding sim at 7 tested distances (1.8-3.32m).
   */
  private void initializeBaselineLUT() {
    double angle = ShotCalculatorConstants.FIXED_LAUNCH_ANGLE_DEG;
    baseLUT.put(1.000, 2570, angle, 0.30);
    baseLUT.put(1.050, 2464, angle, 0.33);
    baseLUT.put(1.100, 2392, angle, 0.35);
    baseLUT.put(1.150, 2339, angle, 0.38);
    baseLUT.put(1.200, 2303, angle, 0.40);
    baseLUT.put(1.250, 2278, angle, 0.42);
    baseLUT.put(1.300, 2260, angle, 0.44);
    baseLUT.put(1.350, 2248, angle, 0.46);
    baseLUT.put(1.400, 2241, angle, 0.48);
    baseLUT.put(1.450, 2238, angle, 0.50);
    baseLUT.put(1.500, 2238, angle, 0.52);
    baseLUT.put(1.550, 2240, angle, 0.54);
    baseLUT.put(1.600, 2245, angle, 0.55);
    baseLUT.put(1.650, 2251, angle, 0.57);
    baseLUT.put(1.700, 2258, angle, 0.58);
    baseLUT.put(1.750, 2267, angle, 0.60);
    baseLUT.put(1.800, 2300, angle, 0.61); // real robot
    baseLUT.put(1.850, 2288, angle, 0.63);
    baseLUT.put(1.900, 2250, angle, 0.66); // real robot
    baseLUT.put(1.950, 2311, angle, 0.66);
    baseLUT.put(2.010, 2225, angle, 0.70); // real robot
    baseLUT.put(2.050, 2337, angle, 0.68);
    baseLUT.put(2.100, 2351, angle, 0.70);
    baseLUT.put(2.150, 2365, angle, 0.71);
    baseLUT.put(2.200, 2379, angle, 0.72);
    baseLUT.put(2.250, 2394, angle, 0.74);
    baseLUT.put(2.300, 2360, angle, 0.76); // real robot
    baseLUT.put(2.350, 2423, angle, 0.76);
    baseLUT.put(2.400, 2438, angle, 0.77);
    baseLUT.put(2.450, 2454, angle, 0.78);
    baseLUT.put(2.500, 2469, angle, 0.80);
    baseLUT.put(2.550, 2485, angle, 0.81);
    baseLUT.put(2.600, 2500, angle, 0.82);
    baseLUT.put(2.654, 2500, angle, 0.84); // real robot
    baseLUT.put(2.700, 2532, angle, 0.84);
    baseLUT.put(2.750, 2548, angle, 0.85);
    baseLUT.put(2.800, 2564, angle, 0.86);
    baseLUT.put(2.850, 2580, angle, 0.87);
    baseLUT.put(2.900, 2700, angle, 0.85); // real robot
    baseLUT.put(2.950, 2613, angle, 0.89);
    baseLUT.put(3.000, 2628, angle, 0.90);
    baseLUT.put(3.050, 2644, angle, 0.91);
    baseLUT.put(3.100, 2660, angle, 0.92);
    baseLUT.put(3.150, 2676, angle, 0.93);
    baseLUT.put(3.200, 2692, angle, 0.94);
    baseLUT.put(3.250, 2709, angle, 0.95);
    baseLUT.put(3.320, 2875, angle, 0.92); // real robot
    baseLUT.put(3.350, 2741, angle, 0.97);
    baseLUT.put(3.400, 2758, angle, 0.98);
    baseLUT.put(3.450, 2773, angle, 0.99);
    baseLUT.put(3.500, 2790, angle, 1.00);
    baseLUT.put(3.550, 2806, angle, 1.01);
    baseLUT.put(3.600, 2822, angle, 1.02);
    baseLUT.put(3.650, 2838, angle, 1.03);
    baseLUT.put(3.700, 2854, angle, 1.04);
    baseLUT.put(3.750, 2870, angle, 1.05);
    baseLUT.put(3.800, 2886, angle, 1.06);
    baseLUT.put(3.850, 2902, angle, 1.06);
    baseLUT.put(3.900, 2918, angle, 1.07);
    baseLUT.put(3.950, 2935, angle, 1.08);
    baseLUT.put(4.000, 2951, angle, 1.09);
    baseLUT.put(4.050, 2966, angle, 1.10);
    baseLUT.put(4.100, 2983, angle, 1.11);
    baseLUT.put(4.150, 2998, angle, 1.12);
    baseLUT.put(4.200, 3014, angle, 1.12);
    baseLUT.put(4.250, 3029, angle, 1.13);
    baseLUT.put(4.300, 3046, angle, 1.14);
    baseLUT.put(4.350, 3062, angle, 1.15);
    baseLUT.put(4.400, 3077, angle, 1.16);
    baseLUT.put(4.450, 3092, angle, 1.17);
    baseLUT.put(4.500, 3108, angle, 1.17);
    baseLUT.put(4.550, 3124, angle, 1.18);
    baseLUT.put(4.600, 3140, angle, 1.19);
    baseLUT.put(4.650, 3156, angle, 1.20);
    baseLUT.put(4.700, 3171, angle, 1.21);
    baseLUT.put(4.750, 3186, angle, 1.21);
    baseLUT.put(4.800, 3202, angle, 1.22);
    baseLUT.put(4.850, 3218, angle, 1.23);
    baseLUT.put(4.900, 3233, angle, 1.24);
    baseLUT.put(4.950, 3249, angle, 1.24);
    baseLUT.put(5.000, 3264, angle, 1.25);
  }

  /** Set swerve reference. Called from RobotContainer during init. */
  public void setSwerve(SwerveSubsystem swerve) {
    this.swerve = swerve;
  }

  // LUT access: baseline + correction overlay + per-distance band multiplier
  double effectiveRPM(double distance) {
    double base = baseLUT.getRPM(distance);
    Double correction = correctionRpmMap.get(distance);
    double raw = base + (correction != null ? correction : 0.0) + rpmOffset;
    return raw * getDistanceBandMultiplier(distance);
  }

  private double getDistanceBandMultiplier(double distance) {
    if (distance < ShotCalculatorConstants.RPM_BAND_SHORT_END) return kRpmMultShort.get();
    if (distance < ShotCalculatorConstants.RPM_BAND_MEDIUM_END) return kRpmMultMedium.get();
    return kRpmMultLong.get();
  }

  String getDistanceBandName(double distance) {
    if (distance < ShotCalculatorConstants.RPM_BAND_SHORT_END) return "SHORT";
    if (distance < ShotCalculatorConstants.RPM_BAND_MEDIUM_END) return "MEDIUM";
    return "LONG";
  }

  double effectiveTOF(double distance) {
    double base = baseLUT.getTOF(distance);
    Double correction = correctionTofMap.get(distance);
    return base + (correction != null ? correction : 0.0);
  }

  double effectiveAngle(double distance) {
    return baseLUT.getAngle(distance);
  }

  /**
   * Drag-compensated effective TOF for velocity offset. The ball's inherited robot velocity decays
   * exponentially during flight. Returns (1 - e^(-c * tof)) / c, reduces to raw tof when c ~ 0.
   */
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

  /** Called once per cycle before CommandScheduler.run(). Caches the result. */
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
    ChassisSpeeds fieldVel = swerve.getFieldVelocity();
    ChassisSpeeds robotVel = swerve.getRobotVelocity();

    if (rawPose == null || fieldVel == null || robotVel == null) {
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

    // second-order pose prediction
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
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

    // Launcher velocity: v_robot_field + omega x r_launcher
    double launcherFieldOffX =
        ShotCalculatorConstants.LAUNCHER_OFFSET_X * cosH
            - ShotCalculatorConstants.LAUNCHER_OFFSET_Y * sinH;
    double launcherFieldOffY =
        ShotCalculatorConstants.LAUNCHER_OFFSET_X * sinH
            + ShotCalculatorConstants.LAUNCHER_OFFSET_Y * cosH;
    double omega = fieldVel.omegaRadiansPerSecond;
    double vx = fieldVel.vxMetersPerSecond + (-launcherFieldOffY) * omega;
    double vy = fieldVel.vyMetersPerSecond + launcherFieldOffX * omega;

    // Capture diagnostics
    diagRobotX = robotX;
    diagRobotY = robotY;
    diagHubX = hubX;
    diagHubY = hubY;
    diagVx = vx;
    diagVy = vy;
    diagOmega = omega;
    diagLauncherOffX = launcherFieldOffX;
    diagLauncherOffY = launcherFieldOffY;

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
    double currentSpeed = robotSpeed;

    speedCapped = robotSpeed > kMaxSOTMSpeed.get();
    if (speedCapped) {
      return LaunchParameters.INVALID;
    }

    velocityFiltered = robotSpeed < kMinSOTMSpeed.get();

    double solvedTOF;
    double projDist;

    if (velocityFiltered) {
      solvedTOF = effectiveTOF(distance);
      projDist = distance;
      iterationsUsed = 0;
      warmStartUsed = false;
    } else {
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
      convergenceCount = 0;

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

        if (convergenceCount < MAX_CONVERGENCE_STEPS) {
          convergenceTOF[convergenceCount] = tof;
          convergenceDist[convergenceCount] = projDist;
          convergenceResidual[convergenceCount] = lookupTOF - tof;
          convergenceCount++;
        }

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

      if (tof > ShotCalculatorConstants.TOF_MAX || tof < 0.0 || Double.isNaN(tof)) {
        tof = effectiveTOF(distance);
        iterationsUsed = (int) kMaxIterations.get() + 1;
      }

      solvedTOF = tof;
    }

    previousTOF = solvedTOF;

    double effectiveTOF = solvedTOF + kMechLatencyMs.get() / 1000.0;

    double effectiveRPMValue = effectiveRPM(projDist);

    // Drive angle: aim at velocity-compensated target position
    double compTargetX;
    double compTargetY;
    if (velocityFiltered) {
      compTargetX = hubX;
      compTargetY = hubY;
      diagRawTOF = solvedTOF;
      diagDragCompTOF = solvedTOF;
    } else {
      double headingDriftTOF = dragCompensatedTOF(solvedTOF);
      compTargetX = hubX - vx * headingDriftTOF;
      compTargetY = hubY - vy * headingDriftTOF;
      diagRawTOF = solvedTOF;
      diagDragCompTOF = headingDriftTOF;
    }
    diagCompTargetX = compTargetX;
    diagCompTargetY = compTargetY;
    diagSolvedTOF = solvedTOF;
    double aimX = compTargetX - robotX;
    double aimY = compTargetY - robotY;
    Rotation2d driveAngle =
        new Rotation2d(aimX, aimY)
            .plus(new Rotation2d(ShotCalculatorConstants.SHOOTER_ANGLE_OFFSET_RAD));

    // copilot aim bias
    double biasInput = aimBiasSupplier.getAsDouble();
    appliedAimBiasDeg = biasInput * kMaxAimBiasDeg.get();
    if (Math.abs(appliedAimBiasDeg) > 0.1) {
      driveAngle = driveAngle.plus(Rotation2d.fromDegrees(appliedAimBiasDeg));
    }

    double headingErrorRad = MathUtil.angleModulus(driveAngle.getRadians() - heading);

    // Angular velocity feedforward
    double driveAngularVelocity = 0;
    if (!velocityFiltered && distance > 0.1) {
      double tangentialVel = (ry * vx - rx * vy) / distance;
      driveAngularVelocity = tangentialVel / distance;
    }
    targetAngularRate = driveAngularVelocity;

    // Direction-aware polar speed limit
    double velocityAngleRad = 0;
    if (robotSpeed > 0.1 && distance > 0.1) {
      double velDot = vx * rx + vy * ry;
      velocityAngleRad = Math.acos(MathUtil.clamp(velDot / (robotSpeed * distance), -1.0, 1.0));
    }
    velocityToHubAngleDeg = Math.toDegrees(velocityAngleRad);

    if (robotSpeed > 0.1) {
      polarSpeedLimitMps =
          computeDirectionalPolarSpeedLimit(
              distance,
              kMaxPolarAngularRate.get(),
              effectiveTOF(distance),
              velocityAngleRad,
              ShotCalculatorConstants.POLAR_SPEED_FLOOR_MPS);
    } else {
      polarSpeedLimitMps = computePolarSpeedLimit(distance, kMaxPolarAngularRate.get());
    }

    // H/V velocity decomposition
    double[] hv = decomposeHV(vx, vy, rx, ry);
    horizontalCompMps = hv[0];
    verticalCompRPM = hv[1] * 60.0;

    double effectiveTolDeg =
        computeAsymmetricTolerance(hv[0], hv[1], kCrossTrackTolDeg.get(), kAlongTrackTolDeg.get());
    crossTrackErrorDeg = Math.abs(Math.toDegrees(headingErrorRad));
    alongTrackErrorDeg = effectiveTolDeg;

    // Confidence scoring
    double solverQuality;
    if (velocityFiltered) {
      solverQuality = 1.0;
    } else {
      int maxIter = (int) kMaxIterations.get();
      if (iterationsUsed > maxIter) {
        solverQuality = 0.0;
      } else if (iterationsUsed <= 3) {
        solverQuality = 1.0;
      } else {
        solverQuality =
            MathUtil.interpolate(1.0, 0.1, (double) (iterationsUsed - 3) / (maxIter - 3));
      }
    }

    double confidence = computeConfidence(solverQuality, currentSpeed, headingErrorRad, distance);

    boolean passing = false;
    boolean isValid = true;

    previousSpeed = currentSpeed;

    double hoodAngle = effectiveAngle(projDist);

    return new LaunchParameters(
        effectiveRPMValue,
        hoodAngle,
        effectiveTOF,
        driveAngle,
        driveAngularVelocity,
        isValid,
        confidence,
        passing);
  }

  /**
   * 5-component weighted geometric mean confidence score (0-100).
   */
  private double computeConfidence(
      double solverQuality, double currentSpeed, double headingErrorRad, double distance) {
    double convergenceQuality = solverQuality;

    double speedDelta = Math.abs(currentSpeed - previousSpeed);
    double velocityStability = MathUtil.clamp(1.0 - speedDelta / 0.5, 0, 1);

    double visionConf;
    try {
      visionConf = ChannelCoordinator.getInstance().getConfidence() / 100.0;
    } catch (Throwable t) {
      visionConf = 0.0;
    }
    visionConf = MathUtil.clamp(visionConf, 0, 1);

    double distanceScale =
        MathUtil.clamp(ShotCalculatorConstants.HEADING_REFERENCE_DISTANCE / distance, 0.5, 2.0);
    double speedScale = 1.0 / (1.0 + ShotCalculatorConstants.HEADING_SPEED_SCALAR * currentSpeed);
    double scaledMaxError =
        ShotCalculatorConstants.HEADING_MAX_ERROR_RAD * distanceScale * speedScale;
    double headingErr = Math.abs(headingErrorRad);
    double headingAccuracy = MathUtil.clamp(1.0 - headingErr / scaledMaxError, 0, 1);

    double rangeSpan =
        ShotCalculatorConstants.MAX_SCORING_DISTANCE - ShotCalculatorConstants.MIN_SCORING_DISTANCE;
    double rangeFraction = (distance - ShotCalculatorConstants.MIN_SCORING_DISTANCE) / rangeSpan;
    double distInRange = 1.0 - 2.0 * Math.abs(rangeFraction - 0.5);
    distInRange = MathUtil.clamp(distInRange, 0, 1);

    double[] c = {convergenceQuality, velocityStability, visionConf, headingAccuracy, distInRange};
    double[] w = {
      ShotCalculatorConstants.W_CONVERGENCE,
      ShotCalculatorConstants.W_VELOCITY_STABILITY,
      ShotCalculatorConstants.W_VISION_CONFIDENCE,
      ShotCalculatorConstants.W_HEADING_ACCURACY,
      ShotCalculatorConstants.W_DISTANCE_IN_RANGE
    };

    double sumW = 0;
    double logSum = 0;
    for (int i = 0; i < 5; i++) {
      if (c[i] <= 0) {
        logConfidenceComponents(convergenceQuality, velocityStability, visionConf, headingAccuracy, distInRange, 0);
        return 0;
      }
      logSum += w[i] * Math.log(c[i]);
      sumW += w[i];
    }
    double composite = Math.exp(logSum / sumW) * 100.0;
    composite = MathUtil.clamp(composite, 0, 100);

    logConfidenceComponents(convergenceQuality, velocityStability, visionConf, headingAccuracy, distInRange, composite);
    return composite;
  }

  private void logConfidenceComponents(
      double convergence, double velStability, double visionConf,
      double headingAcc, double distInRange, double composite) {
    SafeLog.put("Scoring/ShotConfidence/Convergence", convergence);
    SafeLog.put("Scoring/ShotConfidence/VelocityStability", velStability);
    SafeLog.put("Scoring/ShotConfidence/VisionConfidence", visionConf);
    SafeLog.put("Scoring/ShotConfidence/HeadingAccuracy", headingAcc);
    SafeLog.put("Scoring/ShotConfidence/DistanceInRange", distInRange);
    SafeLog.put("Scoring/ShotConfidence", composite);
  }

  // SOTM math helpers

  public static double computePolarSpeedLimit(double distanceM, double maxAngularRateRadPerSec) {
    if (distanceM < 0.5) return 0.5;
    return maxAngularRateRadPerSec * distanceM;
  }

  /**
   * Direction-aware polar speed limit using triangle geometry. Strafing gets a tight cap,
   * approaching gets a generous cap.
   */
  public static double computeDirectionalPolarSpeedLimit(
      double distanceM,
      double maxAngularRateRadPerSec,
      double tofSec,
      double velocityAngleRad,
      double speedFloorMps) {
    if (tofSec <= 0.01 || distanceM < 0.5) {
      return speedFloorMps;
    }

    double hubSweep = MathUtil.clamp(maxAngularRateRadPerSec * tofSec, 0, Math.PI / 2.0);
    double robotAngle = MathUtil.clamp(velocityAngleRad, 0, Math.PI);
    double lookaheadAngle = Math.PI - robotAngle - hubSweep;

    if (lookaheadAngle <= 0.01) {
      return computePolarSpeedLimit(distanceM, maxAngularRateRadPerSec);
    }

    double maxTravel = distanceM * Math.sin(hubSweep) / Math.sin(lookaheadAngle);
    double maxSpeed = maxTravel / tofSec;

    return Math.max(speedFloorMps, maxSpeed);
  }

  /**
   * Decompose velocity into lateral (cross-track) and radial (along-track) relative to hub.
   */
  public static double[] decomposeHV(double vx, double vy, double toHubX, double toHubY) {
    double dist = Math.hypot(toHubX, toHubY);
    if (dist < 0.01) return new double[] {0, 0};
    double ux = toHubX / dist;
    double uy = toHubY / dist;
    double radial = vx * ux + vy * uy;
    double lateral = -vx * uy + vy * ux;
    return new double[] {lateral, radial};
  }

  /**
   * Heading tolerance blended by velocity direction. Strafing = tight, approaching = loose.
   */
  public static double computeAsymmetricTolerance(
      double lateralSpeed, double radialSpeed, double crossTrackTolDeg, double alongTrackTolDeg) {
    double total = Math.abs(lateralSpeed) + Math.abs(radialSpeed);
    if (total < 0.1) return crossTrackTolDeg;
    double lateralFraction = Math.abs(lateralSpeed) / total;
    return crossTrackTolDeg * lateralFraction + alongTrackTolDeg * (1.0 - lateralFraction);
  }

  private Translation2d getHubCenter() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return HubScoringConstants.RED_HUB_CENTER;
    }
    return HubScoringConstants.BLUE_HUB_CENTER;
  }

  private Translation2d getHubForward() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return new Translation2d(-1, 0);
    }
    return new Translation2d(1, 0);
  }

  private void logTelemetry() {
    SafeLog.put("Scoring/ShotCalc/EffectiveRPM", cachedParameters.rpm());
    SafeLog.put("Scoring/ShotCalc/HoodAngleDeg", cachedParameters.hoodAngleDeg());
    SafeLog.put("Scoring/ShotCalc/EffectiveTOF", cachedParameters.timeOfFlightSec());
    SafeLog.put("Scoring/ShotCalc/Distance", solvedDistance);
    SafeLog.put("Scoring/ShotCalc/DriveAngle", cachedParameters.driveAngle().getRadians());
    SafeLog.put(
        "Scoring/ShotCalc/DriveAngularVelocity", cachedParameters.driveAngularVelocityRadPerSec());
    SafeLog.put("Scoring/ShotCalc/IsValid", cachedParameters.isValid());
    SafeLog.put("Scoring/ShotCalc/ConvergenceIterations", iterationsUsed);
    SafeLog.put("Scoring/ShotCalc/WarmStartUsed", warmStartUsed);
    SafeLog.put("Scoring/ShotCalc/VelocityFiltered", velocityFiltered);
    SafeLog.put("Scoring/ShotCalc/BehindHub", behindHub);
    SafeLog.put("Scoring/ShotCalc/SpeedCapped", speedCapped);
    SafeLog.put("Scoring/ShotCalc/RPMOffset", rpmOffset);
    SafeLog.put(
        "Scoring/ShotCalc/DragCompTOF", dragCompensatedTOF(cachedParameters.timeOfFlightSec()));
    SafeLog.put(
        "Scoring/ShotCalc/DistanceBandMultiplier", getDistanceBandMultiplier(solvedDistance));
    SafeLog.put("Scoring/ShotCalc/DistanceBand", getDistanceBandName(solvedDistance));
    SafeLog.put("Scoring/ShotCalc/PolarSpeedLimitMps", polarSpeedLimitMps);
    SafeLog.put("Scoring/ShotCalc/VelocityToHubAngleDeg", velocityToHubAngleDeg);
    SafeLog.put("Scoring/ShotCalc/AimBiasDeg", appliedAimBiasDeg);
    SafeLog.put("Scoring/ShotCalc/TargetAngularRateRadPerSec", targetAngularRate);
    SafeLog.put("Scoring/ShotCalc/HorizontalCompensationMps", horizontalCompMps);
    SafeLog.put("Scoring/ShotCalc/VerticalCompensationRPM", verticalCompRPM);
    SafeLog.put("Scoring/ShotCalc/CrossTrackErrorDeg", crossTrackErrorDeg);
    SafeLog.put("Scoring/ShotCalc/AlongTrackErrorDeg", alongTrackErrorDeg);
    SafeLog.put("Scoring/ShotCalc/RPMOverrideActive", kRpmOverride.get() > 0);
    SafeLog.put("ShotCalc/HubCenterX", getHubCenter().getX());
    SafeLog.put("ShotCalc/HubCenterY", getHubCenter().getY());
    SafeLog.put("ShotCalc/PolarSpeedLimitMps", polarSpeedLimitMps);

    logShotDiagnostics();
  }

  private void logShotDiagnostics() {
    SafeLog.put("Scoring/Diag/RobotVelX", diagVx);
    SafeLog.put("Scoring/Diag/RobotVelY", diagVy);
    double omegaCrossX = -diagLauncherOffY * diagOmega;
    double omegaCrossY = diagLauncherOffX * diagOmega;
    SafeLog.put("Scoring/Diag/OmegaCrossX", omegaCrossX);
    SafeLog.put("Scoring/Diag/OmegaCrossY", omegaCrossY);
    double aimX = diagCompTargetX - diagRobotX;
    double aimY = diagCompTargetY - diagRobotY;
    SafeLog.put("Scoring/Diag/AimVecX", aimX);
    SafeLog.put("Scoring/Diag/AimVecY", aimY);
    double staticAimX = diagHubX - diagRobotX;
    double staticAimY = diagHubY - diagRobotY;
    SafeLog.put("Scoring/Diag/StaticAimX", staticAimX);
    SafeLog.put("Scoring/Diag/StaticAimY", staticAimY);
    SafeLog.put("Scoring/Diag/CompTargetX", diagCompTargetX);
    SafeLog.put("Scoring/Diag/CompTargetY", diagCompTargetY);
    SafeLog.put("Scoring/Diag/HubX", diagHubX);
    SafeLog.put("Scoring/Diag/HubY", diagHubY);
    double aimShiftM = Math.hypot(diagCompTargetX - diagHubX, diagCompTargetY - diagHubY);
    SafeLog.put("Scoring/Diag/AimShiftM", aimShiftM);

    int n = convergenceCount;
    System.arraycopy(convergenceTOF, 0, diagTofTrail, 0, n);
    System.arraycopy(convergenceDist, 0, diagDistTrail, 0, n);
    System.arraycopy(convergenceResidual, 0, diagResidualTrail, 0, n);
    java.util.Arrays.fill(diagTofTrail, n, MAX_CONVERGENCE_STEPS, 0);
    java.util.Arrays.fill(diagDistTrail, n, MAX_CONVERGENCE_STEPS, 0);
    java.util.Arrays.fill(diagResidualTrail, n, MAX_CONVERGENCE_STEPS, 0);
    SafeLog.put("Scoring/Diag/ConvergenceTOF", diagTofTrail);
    SafeLog.put("Scoring/Diag/ConvergenceDist", diagDistTrail);
    SafeLog.put("Scoring/Diag/ConvergenceResidual", diagResidualTrail);
    SafeLog.put("Scoring/Diag/RawTOF", diagRawTOF);
    SafeLog.put("Scoring/Diag/DragCompTOF", diagDragCompTOF);
    SafeLog.put("Scoring/Diag/SolvedTOF", diagSolvedTOF);
    double dragDelta = diagDragCompTOF - diagRawTOF;
    SafeLog.put("Scoring/Diag/DragDeltaSec", dragDelta);
  }

  // Public accessors

  public LaunchParameters getParameters() { return cachedParameters; }
  public double getConfidence() { return cachedParameters.confidence(); }
  public boolean isValid() { return cachedParameters.isValid(); }
  public double getSolvedDistance() { return solvedDistance; }
  public double getPolarSpeedLimitMps() { return polarSpeedLimitMps; }
  public double getTargetAngularRate() { return targetAngularRate; }
  public double getHorizontalCompensationMps() { return horizontalCompMps; }
  public double getVerticalCompensationRPM() { return verticalCompRPM; }
  public double getRpmOverride() { return kRpmOverride.get(); }

  public void setAimBiasSupplier(java.util.function.DoubleSupplier supplier) {
    this.aimBiasSupplier = supplier;
  }

  public double getBaseRPM(double distance) { return baseLUT.getRPM(distance); }
  public void addRpmCorrection(double distance, double deltaRpm) { correctionRpmMap.put(distance, deltaRpm); }
  public void addTofCorrection(double distance, double deltaTof) { correctionTofMap.put(distance, deltaTof); }
  public void clearCorrections() { correctionRpmMap.clear(); correctionTofMap.clear(); }

  public void adjustOffset(double delta) {
    rpmOffset = MathUtil.clamp(rpmOffset + delta,
        -ShotCalculatorConstants.RPM_OFFSET_MAX, ShotCalculatorConstants.RPM_OFFSET_MAX);
  }

  public void resetOffset() { rpmOffset = 0; }
  public double getOffset() { return rpmOffset; }
  public void resetState() { previousTOF = -1; previousSpeed = 0; }

  public double getTimeOfFlight(double distanceM) { return effectiveTOF(distanceM); }
  public double getMinTimeOfFlight() { return effectiveTOF(ShotCalculatorConstants.MIN_SCORING_DISTANCE); }
  public double getMaxTimeOfFlight() { return effectiveTOF(ShotCalculatorConstants.MAX_SCORING_DISTANCE); }

  // diagnostic accessors
  public double getDiagRobotX() { return diagRobotX; }
  public double getDiagRobotY() { return diagRobotY; }
  public double getDiagHubX() { return diagHubX; }
  public double getDiagHubY() { return diagHubY; }
  public double getDiagCompTargetX() { return diagCompTargetX; }
  public double getDiagCompTargetY() { return diagCompTargetY; }
  public double getDiagSolvedTOF() { return diagSolvedTOF; }

  ShotLUT getBaseLUT() { return baseLUT; }
}
