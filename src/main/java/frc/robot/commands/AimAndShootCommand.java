package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.HubScoringConstants;
import frc.robot.Constants.ShotCalculatorConstants;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.telemetry.SafeLog;
import frc.robot.util.HeadingController;
import frc.robot.util.ShotCalculator;
import frc.robot.util.TunableNumber;
import java.util.function.DoubleSupplier;

/**
 * Points the rear at the hub, spins up, and feeds balls when at speed. Driver keeps translation
 * control. In auto mode it times out; in teleop it runs until the operator lets go of RT.
 */
public class AimAndShootCommand extends Command {

  private final SwerveSubsystem swerve;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Agitator agitator;
  private final DoubleSupplier forwardInput;
  private final DoubleSupplier strafeInput;
  private final boolean autoFinish;

  private final HeadingController headingController = new HeadingController();
  private final Timer autoTimer = new Timer();
  private final Timer reverseTimer = new Timer();

  // Firing hysteresis state
  private boolean reachedSpeed = false;
  private boolean feeding = false;
  private double underShootingSince = -1;

  // cached to avoid GC pressure in execute()
  private ChassisSpeeds reusableSpeeds = new ChassisSpeeds();

  // blend COR from launcher (precise) to robot center (fast) based on heading error
  private static final double COR_MIN_ERROR_RAD = Math.toRadians(2.0);
  private static final double COR_MAX_ERROR_RAD = Math.toRadians(15.0);
  private static final Translation2d LAUNCHER_OFFSET =
      new Translation2d(ShotCalculatorConstants.LAUNCHER_OFFSET_X, 0);

  // Tunable thresholds matching SpeedUpThenIndex
  private static final TunableNumber autoTimeoutSec =
      new TunableNumber("AimAndShoot/AutoTimeoutSec", 4.5);
  private static final TunableNumber underShootPct =
      new TunableNumber("AimAndShoot/UnderShootPct", 0.85);
  private static final TunableNumber underShootDebounceMs =
      new TunableNumber("AimAndShoot/DebounceMs", 400);
  private static final TunableNumber feedRatioFloor =
      new TunableNumber("AimAndShoot/FeedRatioFloor", 0.5);
  // private static final TunableNumber reverseDurationMs =
  //     new TunableNumber("AimAndShoot/ReverseDurationMs", 650);
  private static final TunableNumber omegaLimit = new TunableNumber("AimAndShoot/OmegaLimit", 0.4);

  // X-lock wheels when driver isn't moving to prevent drift while shooting
  private static final double OLOCK_TRANSLATION_THRESHOLD = 0.1;
  private static final double OLOCK_OMEGA_THRESHOLD = 0.15;

  public AimAndShootCommand(
      SwerveSubsystem swerve,
      Shooter shooter,
      Indexer indexer,
      Agitator agitator,
      DoubleSupplier forwardInput,
      DoubleSupplier strafeInput,
      boolean autoFinish) {
    this.swerve = swerve;
    this.shooter = shooter;
    this.indexer = indexer;
    this.agitator = agitator;
    this.forwardInput = forwardInput;
    this.strafeInput = strafeInput;
    this.autoFinish = autoFinish;

    addRequirements(swerve, shooter, indexer, agitator);
  }

  @Override
  public void initialize() {
    // reverse to clear jammed balls before feeding, adjust RPM if too aggressive
    // agitator.runVelocity();
    indexer.moveToVelocityWithPID(-5000);
    reverseTimer.restart();

    headingController.reset();
    reachedSpeed = false;
    feeding = false;
    underShootingSince = -1;

    // start spinning early with flat RPM, execute() will switch to LUT RPM
    shooter.moveToVelocityWithPID(1000);

    if (autoFinish) {
      autoTimer.restart();
    }
  }

  @Override
  public void execute() {
    // aim: point rear at hub
    ShotCalculator shotCalc = ShotCalculator.getInstance();
    ShotCalculator.LaunchParameters params = shotCalc.getParameters();
    Rotation2d targetHeading = params.isValid() ? params.driveAngle() : computeFallbackAim();

    double maxOmega = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();
    double distToHub = swerve.getPose().getTranslation().getDistance(getHubCenter());

    double omega =
        headingController.calculate(
            swerve.getHeading(),
            targetHeading,
            params.isValid() ? params.driveAngularVelocityRadPerSec() : 0,
            distToHub,
            maxOmega * omegaLimit.get());

    // drive: let the driver translate while we control heading
    double maxVel = swerve.getSwerveDrive().getMaximumChassisVelocity();
    double fwd = MathUtil.applyDeadband(forwardInput.getAsDouble(), 0.08) * maxVel;
    double str = MathUtil.applyDeadband(strafeInput.getAsDouble(), 0.08) * maxVel;

    // cap driver speed so we don't outrun the aim compensation
    if (params.isValid()) {
      double polarLimit = shotCalc.getPolarSpeedLimitMps();
      if (polarLimit > 0 && polarLimit < maxVel) {
        double speed = Math.hypot(fwd, str);
        if (speed > polarLimit) {
          double scale = polarLimit / speed;
          fwd *= scale;
          str *= scale;
        }
      }
    }

    // X-lock if we're basically standing still so we don't drift while shooting
    if (Math.hypot(fwd, str) < OLOCK_TRANSLATION_THRESHOLD
        && Math.abs(omega) < OLOCK_OMEGA_THRESHOLD) {
      swerve.lock();
    } else {
      // blend COR: launcher offset when close to aimed, robot center when whipping around
      double headingErrorRad = Math.abs(targetHeading.minus(swerve.getHeading()).getRadians());
      double corScalar =
          MathUtil.clamp(
              (headingErrorRad - COR_MIN_ERROR_RAD) / (COR_MAX_ERROR_RAD - COR_MIN_ERROR_RAD),
              0,
              1);
      Translation2d cor = LAUNCHER_OFFSET.times(1.0 - corScalar);

      reusableSpeeds.vxMetersPerSecond = fwd;
      reusableSpeeds.vyMetersPerSecond = str;
      reusableSpeeds.omegaRadiansPerSecond = omega;
      swerve.drive(reusableSpeeds, cor);
    }

    // RPMOverride takes priority, then LUT RPM, then flat dashboard RPM
    double overrideRPM = shotCalc.getRpmOverride();
    double targetRPM;
    if (overrideRPM > 0) {
      targetRPM = overrideRPM;
    } else if (params.isValid()) {
      targetRPM = params.rpm();
    } else {
      targetRPM = shooter.getTunableTargetRPM();
    }
    shooter.moveToVelocityWithPID(targetRPM);

    // still in reverse phase, keep clearing balls while shooter spins up
    // if (!reverseTimer.hasElapsed(reverseDurationMs.get() / 1000.0)) {
    //   return;
    // }

    boolean atSpeed = shooter.isAtSpeed(targetRPM);
    if (atSpeed && !reachedSpeed) {
      reachedSpeed = true;
      feeding = true;
    }

    // if RPM tanks for too long, stop feeding until it recovers
    if (reachedSpeed) {
      double velocity = shooter.getVelocityRPM();
      double target = shooter.getTargetRPM();
      double threshold = target * underShootPct.get();

      if (velocity < threshold) {
        double now = Timer.getFPGATimestamp();
        if (underShootingSince < 0) {
          underShootingSince = now;
        } else if ((now - underShootingSince) * 1000.0 > underShootDebounceMs.get()) {
          feeding = false;
        }
      } else {
        underShootingSince = -1;
        if (!feeding) {
          feeding = true;
        }
      }
    }

    if (feeding) {
      double targetRpm = shooter.getTargetRPM();
      // double rpmRatio = (targetRpm > 0) ? Math.min(1.0, shooter.getVelocityRPM() / targetRpm) :
      // 0;
      // rpmRatio = Math.max(feedRatioFloor.get(), rpmRatio);
      indexer.moveToVelocityWithPID(indexer.getTunableTargetSpeed());
      agitator.runVelocity();
      // } else {
      //   indexer.move(0);
      //   agitator.moveToVelocityWithPID(agitator.getTunableTargetRPM() * 0.1);
    } else {
      agitator.stopVelocity();
    }

    // progressive aim haptic: operator feels heading error converge
    try {
      double errorDeg = Math.toDegrees(Math.abs(headingController.getPositionError()));
      frc.robot.util.DriverFeedback.getInstance().setProgressiveAim(errorDeg);
    } catch (Throwable t) {
    }

    // telemetry
    SafeLog.put("AimAndShoot/Active", true);
    SafeLog.put("AimAndShoot/Feeding", feeding);
    if (autoFinish) {
      SafeLog.put("AimAndShoot/AutoTimerSec", autoTimer.get());
    }
    if (Constants.TUNING_MODE) {
      SafeLog.put(
          "AimAndShoot/HeadingErrorDeg", Math.toDegrees(headingController.getPositionError()));
      SafeLog.put("AimAndShoot/ShooterAtSpeed", reachedSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.move(0);
    indexer.move(0);
    agitator.stopVelocity();
    try {
      frc.robot.util.DriverFeedback.getInstance().clearProgressiveAim();
    } catch (Throwable t) {
    }
    SafeLog.put("AimAndShoot/Active", false);
  }

  @Override
  public boolean isFinished() {
    if (autoFinish) {
      return autoTimer.hasElapsed(autoTimeoutSec.get());
    }
    return false;
  }

  /** Simple angle-to-hub when ShotCalculator can't solve (out of range, behind hub, etc). */
  private Rotation2d computeFallbackAim() {
    Translation2d hub = getHubCenter();
    Translation2d robot = swerve.getPose().getTranslation();
    if (Double.isNaN(robot.getX()) || Double.isNaN(robot.getY())) {
      return swerve.getHeading(); // hold current heading if pose is corrupt
    }
    // rear-mounted shooter: aim the back of the robot at the hub
    return robot
        .minus(hub)
        .getAngle()
        .plus(new Rotation2d(ShotCalculatorConstants.SHOOTER_ANGLE_OFFSET_RAD));
  }

  private Translation2d getHubCenter() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return HubScoringConstants.RED_HUB_CENTER;
    }
    return HubScoringConstants.BLUE_HUB_CENTER;
  }
}
