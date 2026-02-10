package frc.robot.telemetry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.telemetry.SwerveDriveTelemetry;

/** Drive telemetry: pose, module states, chassis speeds, gyro, encoder health. */
public class DriveTelemetry implements SubsystemTelemetry {
  private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};
  private static final double ENCODER_DISAGREEMENT_THRESHOLD_RAD = 0.05;

  private SwerveSubsystem swerveSubsystem;
  private SwerveDrive swerveDrive;
  private boolean subsystemAvailable = false;

  // Current state
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

  // Odometry timestamp for jitter analysis
  private double odometryTimestampSec = 0;

  // Encoder health per module
  private boolean[] encoderAbsoluteOk = {true, true, true, true};
  private double[] encoderDisagreementRad = {0, 0, 0, 0};

  private double maxVelocityMPS = 0;

  // Auto path tracking
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

      // Get module setpoints (desired states) from YAGSL internal telemetry
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

      // Get requested chassis speeds from YAGSL internal telemetry
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

      // Encoder health per module
      updateEncoderHealth();

      // Max chassis velocity from YAGSL
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

  private void updateEncoderHealth() {
    try {
      SwerveModule[] modules = swerveDrive.getModules();
      if (modules == null) return;

      for (int i = 0; i < Math.min(modules.length, 4); i++) {
        SwerveModule mod = modules[i];
        if (mod == null) continue;

        try {
          // Check if absolute encoder is responding
          double absAngle = mod.getAbsolutePosition();
          encoderAbsoluteOk[i] = !Double.isNaN(absAngle);

          // Compare absolute vs relative encoder angles
          double relAngle = mod.getRelativePosition();
          double disagreement = Math.abs(MathUtil.angleModulus(absAngle - relAngle));
          encoderDisagreementRad[i] = disagreement;
        } catch (Throwable t) {
          encoderAbsoluteOk[i] = false;
          encoderDisagreementRad[i] = 0;
        }
      }
    } catch (Throwable t) {
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

    // Odometry timestamp for jitter analysis
    SafeLog.put("Drive/OdometryTimestampSec", odometryTimestampSec);

    // Module setpoints (commanded vs measured)
    for (int i = 0; i < 4; i++) {
      String name = MODULE_NAMES[i];
      if (moduleSetpoints[i] != null) {
        SafeLog.put(
            "Drive/ModuleSetpoints/" + name + "/Angle", moduleSetpoints[i].angle.getRadians());
        SafeLog.put(
            "Drive/ModuleSetpoints/" + name + "/Speed", moduleSetpoints[i].speedMetersPerSecond);
      }
    }

    // Encoder health per module
    for (int i = 0; i < 4; i++) {
      String name = MODULE_NAMES[i];
      SafeLog.put("Drive/Encoder/" + name + "/AbsoluteOk", encoderAbsoluteOk[i]);
      // Only log disagreement if above threshold (reduce noise)
      if (encoderDisagreementRad[i] > ENCODER_DISAGREEMENT_THRESHOLD_RAD) {
        SafeLog.put("Drive/Encoder/" + name + "/DisagreementRad", encoderDisagreementRad[i]);
      } else {
        SafeLog.put("Drive/Encoder/" + name + "/DisagreementRad", 0.0);
      }
    }

    // Auto path tracking
    SafeLog.put("Drive/Auto/TargetPose", targetPose);
    SafeLog.put("Drive/Auto/PositionErrorM", positionErrorMeters);
    SafeLog.put("Drive/Auto/HeadingErrorDeg", headingErrorDegrees);
    SafeLog.put("Drive/Auto/IsFollowing", isFollowingPath);
    SafeLog.put("Drive/Auto/PathName", currentPathName);
  }

  @Override
  public String getName() {
    return "Drive";
  }

  // ===== Auto Path Tracking Methods =====

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

  // Accessors
  public boolean isFollowingPath() {
    return isFollowingPath;
  }

  public double getPositionErrorMeters() {
    return positionErrorMeters;
  }

  public double getHeadingErrorDegrees() {
    return headingErrorDegrees;
  }
}
