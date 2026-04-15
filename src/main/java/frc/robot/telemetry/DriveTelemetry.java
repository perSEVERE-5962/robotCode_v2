package frc.robot.telemetry;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.telemetry.SwerveDriveTelemetry;

/** Drive telemetry: pose, module states, chassis speeds, gyro, encoder health. */
public class DriveTelemetry implements SubsystemTelemetry {
  private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

  // 10 degrees in radians. Picked against the real steady-state noise floor,
  // where a healthy module sits under 0.05 rad and a settling transient briefly
  // touches 0.15 rad. The debounce below catches sustained drift while ignoring
  // the short post-command oscillation.
  private static final double ENCODER_DISAGREEMENT_THRESHOLD_RAD = 0.175;

  // A real calibration event stays high for seconds, not milliseconds.
  private static final double ENCODER_FAULT_DEBOUNCE_SEC = 1.0;
  private static final double LOOP_PERIOD_SEC = 0.02;

  private SwerveSubsystem swerveSubsystem;
  private SwerveDrive swerveDrive;
  private boolean subsystemAvailable = false;

  private Pose2d pose = new Pose2d();
  private SwerveModuleState[] moduleStates = new SwerveModuleState[4];
  private SwerveModuleState[] moduleSetpoints = new SwerveModuleState[4];
  private double vxMetersPerSec = 0;
  private double vyMetersPerSec = 0;
  private double omegaRadPerSec = 0;
  private double requestedVx = 0;
  private double requestedVy = 0;
  private double requestedOmega = 0;
  private double yawRad = 0;
  private double pitchRad = 0;
  private double rollRad = 0;
  private boolean gyroConnected = false;

  private double odometryTimestampSec = 0;

  private boolean[] encoderAbsoluteOk = {true, true, true, true};
  private double[] encoderDisagreementRad = {0, 0, 0, 0};
  private double[] encoderFaultDebounceSec = {0, 0, 0, 0};
  private boolean[] encoderFaultLatched = {false, false, false, false};

  private static final int SWERVE_HEALTH_DECIMATION = 25;
  private int swerveHealthCounter = 0;
  private boolean[] driveMotorConnected = {false, false, false, false};
  private boolean[] turnMotorConnected = {false, false, false, false};
  private boolean[] encoderReadIssue = {true, true, true, true};
  private int[] driveFaultsRaw = {0, 0, 0, 0};
  private int[] turnFaultsRaw = {0, 0, 0, 0};
  private double[] driveTemperature = {0, 0, 0, 0};

  private double maxVelocityMPS = 0;

  private Pose2d targetPose = new Pose2d();
  private double positionErrorMeters = 0;
  private double headingErrorDegrees = 0;
  private boolean isFollowingPath = false;
  private String currentPathName = "none";

  public DriveTelemetry() {
    for (int i = 0; i < 4; i++) {
      moduleStates[i] = new SwerveModuleState(0, new Rotation2d());
      moduleSetpoints[i] = new SwerveModuleState(0, new Rotation2d());
    }
  }

  /** Called from RobotContainer. */
  public void setSwerveSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    if (swerveSubsystem != null) {
      try {
        this.swerveDrive = swerveSubsystem.getSwerveDrive();
      } catch (Throwable t) {
        this.swerveDrive = null;
      }
    }
  }

  @Override
  public void update() {
    odometryTimestampSec = Timer.getFPGATimestamp();

    if (swerveDrive == null) {
      subsystemAvailable = false;
      setDefaultValues();
      return;
    }

    subsystemAvailable = true;

    try {
      Pose2d newPose = swerveDrive.getPose();
      pose = (newPose != null) ? newPose : new Pose2d();

      SwerveModuleState[] newStates = swerveDrive.getStates();
      if (newStates != null && newStates.length == 4) {
        moduleStates = newStates;
      }

      try {
        SwerveModuleState[] desiredStates = SwerveDriveTelemetry.desiredStatesObj;
        if (desiredStates != null && desiredStates.length >= 4) {
          for (int i = 0; i < 4; i++) {
            if (desiredStates[i] != null) {
              moduleSetpoints[i] = desiredStates[i];
            }
          }
        }
      } catch (Throwable t) {
        // Fall back to measured states if YAGSL internal API unavailable
      }

      try {
        ChassisSpeeds desiredSpeeds = SwerveDriveTelemetry.desiredChassisSpeedsObj;
        if (desiredSpeeds != null) {
          requestedVx = desiredSpeeds.vxMetersPerSecond;
          requestedVy = desiredSpeeds.vyMetersPerSecond;
          requestedOmega = desiredSpeeds.omegaRadiansPerSecond;
        }
      } catch (Throwable t) {
        // YAGSL internal API may not expose this field
      }

      ChassisSpeeds speeds = swerveDrive.getRobotVelocity();
      if (speeds != null) {
        vxMetersPerSec = speeds.vxMetersPerSecond;
        vyMetersPerSec = speeds.vyMetersPerSecond;
        omegaRadPerSec = speeds.omegaRadiansPerSecond;
      }

      var yaw = swerveDrive.getYaw();
      var pitch = swerveDrive.getPitch();
      var roll = swerveDrive.getRoll();
      yawRad = (yaw != null) ? yaw.getRadians() : 0;
      pitchRad = (pitch != null) ? pitch.getRadians() : 0;
      rollRad = (roll != null) ? roll.getRadians() : 0;
      gyroConnected = (yaw != null);

      updateEncoderHealth();
      updateSwerveMotorHealth();

      try {
        maxVelocityMPS = swerveDrive.getMaximumChassisVelocity();
      } catch (Throwable t) {
        // YAGSL API may not expose this
      }

      if (isFollowingPath && targetPose != null && pose != null) {
        double dx = targetPose.getX() - pose.getX();
        double dy = targetPose.getY() - pose.getY();
        positionErrorMeters = Math.sqrt(dx * dx + dy * dy);
        var rotation = targetPose.getRotation().minus(pose.getRotation());
        headingErrorDegrees = (rotation != null) ? Math.abs(rotation.getDegrees()) : 0;
      }
    } catch (Throwable t) {
      subsystemAvailable = false;
    }
  }

  /**
   * Pure math for the encoder disagreement signal. Takes absolute and relative angles in DEGREES
   * (as YAGSL reports them), returns the short-arc disagreement in radians. NaN on either input
   * short-circuits to 0 so no garbage values can leak into the log.
   *
   * <p>Public-package for unit testing. Do not call from hot paths other than {@link
   * #updateEncoderHealth()} because it allocates nothing and the test uses it directly.
   */
  static double computeDisagreementRad(double absAngleDeg, double relAngleDeg) {
    if (Double.isNaN(absAngleDeg) || Double.isNaN(relAngleDeg)) {
      return 0.0;
    }
    double absRad = Math.toRadians(absAngleDeg);
    double relRad = Math.toRadians(relAngleDeg);
    return Math.abs(MathUtil.angleModulus(absRad - relRad));
  }

  private void updateEncoderHealth() {
    try {
      SwerveModule[] modules = swerveDrive.getModules();
      if (modules == null) return;

      for (int i = 0; i < Math.min(modules.length, 4); i++) {
        SwerveModule mod = modules[i];
        if (mod == null) continue;

        try {
          double absAngleDeg = mod.getAbsolutePosition();
          double relAngleDeg = mod.getRelativePosition();
          boolean absOk = !Double.isNaN(absAngleDeg);
          encoderAbsoluteOk[i] = absOk;

          double disagreement = computeDisagreementRad(absAngleDeg, relAngleDeg);
          encoderDisagreementRad[i] = disagreement;

          // Debounce: a real drift stays high across many cycles. A settling
          // transient clears on the next cycle. Latch only after the fault
          // window fills up, so short spikes don't fire the alert.
          if (absOk && disagreement > ENCODER_DISAGREEMENT_THRESHOLD_RAD) {
            encoderFaultDebounceSec[i] += LOOP_PERIOD_SEC;
          } else {
            encoderFaultDebounceSec[i] = 0;
          }
          encoderFaultLatched[i] = encoderFaultDebounceSec[i] >= ENCODER_FAULT_DEBOUNCE_SEC;
        } catch (Throwable t) {
          encoderAbsoluteOk[i] = false;
          encoderDisagreementRad[i] = 0;
          encoderFaultDebounceSec[i] = 0;
          encoderFaultLatched[i] = false;
        }
      }
    } catch (Throwable t) {
    }
  }

  private void updateSwerveMotorHealth() {
    swerveHealthCounter++;
    if (swerveHealthCounter < SWERVE_HEALTH_DECIMATION) return;
    swerveHealthCounter = 0;

    try {
      SwerveModule[] modules = swerveDrive.getModules();
      if (modules == null) return;

      for (int i = 0; i < Math.min(modules.length, 4); i++) {
        SwerveModule mod = modules[i];
        if (mod == null) continue;

        try {
          Object driveMotorObj = mod.getDriveMotor().getMotor();
          if (driveMotorObj instanceof SparkBase) {
            SparkBase spark = (SparkBase) driveMotorObj;
            SparkBase.Faults faults = spark.getFaults();
            driveMotorConnected[i] = !faults.can;
            driveFaultsRaw[i] = faults.rawBits;
            driveTemperature[i] = spark.getMotorTemperature();
          }
        } catch (Throwable t) {
          driveMotorConnected[i] = false;
          driveFaultsRaw[i] = -1;
        }

        try {
          Object turnMotorObj = mod.getAngleMotor().getMotor();
          if (turnMotorObj instanceof SparkBase) {
            SparkBase spark = (SparkBase) turnMotorObj;
            SparkBase.Faults faults = spark.getFaults();
            turnMotorConnected[i] = !faults.can;
            turnFaultsRaw[i] = faults.rawBits;
          }
        } catch (Throwable t) {
          turnMotorConnected[i] = false;
          turnFaultsRaw[i] = -1;
        }

        try {
          encoderReadIssue[i] = mod.getAbsoluteEncoderReadIssue();
        } catch (Throwable t) {
          encoderReadIssue[i] = true;
        }
      }
    } catch (Throwable t) {
      // SwerveDrive not ready
    }
  }

  private void setDefaultValues() {
    pose = new Pose2d();
    vxMetersPerSec = 0;
    vyMetersPerSec = 0;
    omegaRadPerSec = 0;
    yawRad = 0;
    pitchRad = 0;
    rollRad = 0;
    for (int i = 0; i < 4; i++) {
      driveMotorConnected[i] = false;
      turnMotorConnected[i] = false;
      encoderReadIssue[i] = true;
      driveFaultsRaw[i] = 0;
      turnFaultsRaw[i] = 0;
      driveTemperature[i] = 0;
    }
  }

  @Override
  public void log() {
    SafeLog.put("Drive/Available", subsystemAvailable);
    SafeLog.put("Drive/Pose", pose);
    SafeLog.put("Drive/ModuleStates/Measured", moduleStates);
    SafeLog.put("Drive/ChassisSpeeds/vXMetersPerSec", vxMetersPerSec);
    SafeLog.put("Drive/ChassisSpeeds/vYMetersPerSec", vyMetersPerSec);
    SafeLog.put("Drive/ChassisSpeeds/omegaRadPerSec", omegaRadPerSec);
    SafeLog.put("Drive/ChassisSpeeds/Requested/vXMetersPerSec", requestedVx);
    SafeLog.put("Drive/ChassisSpeeds/Requested/vYMetersPerSec", requestedVy);
    SafeLog.put("Drive/ChassisSpeeds/Requested/omegaRadPerSec", requestedOmega);
    SafeLog.put("Drive/Gyro/YawRad", yawRad);
    SafeLog.put("Drive/Gyro/PitchRad", pitchRad);
    SafeLog.put("Drive/Gyro/RollRad", rollRad);
    SafeLog.put("Drive/Gyro/Connected", gyroConnected);

    SafeLog.put("Drive/MaxVelocityMPS", maxVelocityMPS);

    SafeLog.put("Drive/OdometryTimestampSec", odometryTimestampSec);

    for (int i = 0; i < 4; i++) {
      String name = MODULE_NAMES[i];
      SafeLog.put("Drive/Module/" + name + "/DriveConnected", driveMotorConnected[i]);
      SafeLog.put("Drive/Module/" + name + "/TurnConnected", turnMotorConnected[i]);
    }

    SafeLog.put("Drive/Auto/IsFollowing", isFollowingPath);
    SafeLog.put("Drive/Auto/PathName", currentPathName);

    // Always on. These four arrays are tiny and catching a module drift
    // matters every match, not just when tuning. The log bandwidth cost
    // is under 0.1 percent of the total stream.
    for (int i = 0; i < 4; i++) {
      String name = MODULE_NAMES[i];
      SafeLog.put("Drive/Encoder/" + name + "/AbsoluteOk", encoderAbsoluteOk[i]);
      SafeLog.put("Drive/Encoder/" + name + "/DisagreementRad", encoderDisagreementRad[i]);
      SafeLog.put("Drive/Encoder/" + name + "/EncoderIssue", encoderFaultLatched[i]);
    }

    // Debug-only signals gated behind TUNING_MODE to reduce CAN/log bandwidth in competition
    if (Constants.TUNING_MODE) {
      for (int i = 0; i < 4; i++) {
        String name = MODULE_NAMES[i];
        if (moduleSetpoints[i] != null) {
          SafeLog.put(
              "Drive/ModuleSetpoints/" + name + "/Angle", moduleSetpoints[i].angle.getRadians());
          SafeLog.put(
              "Drive/ModuleSetpoints/" + name + "/Speed", moduleSetpoints[i].speedMetersPerSecond);
        }
      }

      for (int i = 0; i < 4; i++) {
        String name = MODULE_NAMES[i];
        SafeLog.put("Drive/Module/" + name + "/DriveFaultsRaw", driveFaultsRaw[i]);
        SafeLog.put("Drive/Module/" + name + "/DriveTemperature", driveTemperature[i]);
        SafeLog.put("Drive/Module/" + name + "/TurnFaultsRaw", turnFaultsRaw[i]);
        SafeLog.put("Drive/Module/" + name + "/EncoderIssue", encoderReadIssue[i]);
      }

      SafeLog.put("Drive/Auto/TargetPose", targetPose);
      SafeLog.put("Drive/Auto/PositionErrorM", positionErrorMeters);
      SafeLog.put("Drive/Auto/HeadingErrorDeg", headingErrorDegrees);
    }
  }

  @Override
  public String getName() {
    return "Drive";
  }

  public void setTargetPose(Pose2d target) {
    this.targetPose = (target != null) ? target : new Pose2d();
    this.isFollowingPath = true;
  }

  public void setPathName(String name) {
    this.currentPathName = (name != null) ? name : "none";
  }

  public void pathComplete() {
    this.isFollowingPath = false;
    this.positionErrorMeters = 0;
    this.headingErrorDegrees = 0;
  }

  public boolean isFollowingPath() {
    return isFollowingPath;
  }

  public double getPositionErrorMeters() {
    return positionErrorMeters;
  }

  public double getHeadingErrorDegrees() {
    return headingErrorDegrees;
  }

  public boolean isGyroConnected() {
    return gyroConnected;
  }

  public boolean isDriveMotorConnected(int module) {
    return (module >= 0 && module < 4) && driveMotorConnected[module];
  }

  public boolean isTurnMotorConnected(int module) {
    return (module >= 0 && module < 4) && turnMotorConnected[module];
  }

  public boolean isEncoderOk(int module) {
    return (module >= 0 && module < 4) && !encoderReadIssue[module];
  }

  public int getDriveFaultsRaw(int module) {
    return (module >= 0 && module < 4) ? driveFaultsRaw[module] : 0;
  }

  public int getTurnFaultsRaw(int module) {
    return (module >= 0 && module < 4) ? turnFaultsRaw[module] : 0;
  }
}
