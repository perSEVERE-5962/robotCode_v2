package frc.robot.telemetry;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Cameras;
import frc.robot.subsystems.swervedrive.Vision;

/** Vision telemetry: target lock, tag ID stability, pose confidence. */
public class VisionTelemetry implements SubsystemTelemetry {
  private Vision vision;
  private boolean subsystemAvailable = false;
  private boolean visionHealthy = true;

  private static final int LOCK_THRESHOLD_FRAMES = 5;
  private static final int STABLE_LOCK_THRESHOLD_FRAMES = 10;

  // Current state
  private boolean hasTarget = false;
  private boolean lockedOnTarget = false;
  private int consecutiveFrames = 0;
  private double lastTargetTimestamp = 0;
  private double timeSinceLastTargetMs = 0;

  // Tag ID stability tracking
  private int lastTagID = -1;
  private int sameTagFrameCount = 0;
  private int lockedTagID = -1;
  private boolean stableLock = false;

  // For CycleTracker wiring
  private boolean wasLockedForCycle = false;

  // Quality metrics
  private double distanceToTargetM = 0;
  private double poseConfidence = 0;
  private double lockDurationMs = 0;
  private double lockStartTime = 0;
  private boolean wasLocked = false;

  // Pose measurement timing and trust
  private double poseTimestampSec = 0;
  private double[] measurementStdDevs = {0.5, 0.5, 0.5}; // x, y, theta defaults

  // Target orientation and pipeline health
  private double targetYawDeg = 0;
  private double targetPitchDeg = 0;
  private double latencyMs = 0;
  private boolean leftCamConnected = false;
  private boolean rightCamConnected = false;

  private String bestCameraName = "NONE";
  private int camerasWithTarget = 0;

  public VisionTelemetry() {}

  /** Called from RobotContainer */
  public void setVision(Vision vision) {
    this.vision = vision;
  }

  private double sanitize(double value) {
    return Double.isFinite(value) ? value : 0.0;
  }

  @Override
  public void update() {
    if (vision == null) {
      subsystemAvailable = false;
      setDefaultValues();
      return;
    }

    subsystemAvailable = true;
    double now = Timer.getFPGATimestamp();

    try {
      hasTarget = vision.hasTarget();

      if (hasTarget) {
        consecutiveFrames++;
        lastTargetTimestamp = now;
      } else {
        consecutiveFrames = 0;
      }

      lockedOnTarget = consecutiveFrames >= LOCK_THRESHOLD_FRAMES;
      timeSinceLastTargetMs = sanitize((now - lastTargetTimestamp) * 1000.0);

      // CycleTracker wiring: notify when lock acquired
      if (lockedOnTarget && !wasLockedForCycle) {
        SafeLog.run(() -> CycleTracker.getInstance().aimComplete());
      }
      wasLockedForCycle = lockedOnTarget;

      // Tag ID stability tracking
      int currentTagID = vision.getBestTargetId();

      if (currentTagID == lastTagID && currentTagID != -1) {
        sameTagFrameCount++;
      } else {
        sameTagFrameCount = (currentTagID != -1) ? 1 : 0;
      }
      lastTagID = currentTagID;

      // L1: Only assign lockedTagID when actually locked
      if (lockedOnTarget && currentTagID != -1) {
        lockedTagID = currentTagID;
      } else if (!lockedOnTarget) {
        lockedTagID = -1; // Clear when not locked
      }
      stableLock = sameTagFrameCount >= STABLE_LOCK_THRESHOLD_FRAMES;

      // Quality metrics with NaN protection
      if (currentTagID != -1) {
        distanceToTargetM = sanitize(vision.getDistanceFromAprilTag(currentTagID));
      } else {
        distanceToTargetM = 0;
      }

      poseConfidence = computeConfidence(distanceToTargetM, sameTagFrameCount, stableLock);

      // Track lock duration
      if (lockedOnTarget && !wasLocked) {
        lockStartTime = now;
      }
      if (lockedOnTarget) {
        lockDurationMs = sanitize((now - lockStartTime) * 1000);
      } else {
        lockDurationMs = 0;
      }
      wasLocked = lockedOnTarget;

      // Pose timestamp and std devs
      // Use last target timestamp as proxy (actual pose timestamp requires API update)
      poseTimestampSec = hasTarget ? lastTargetTimestamp : now;

      // Std devs based on distance (closer = more trust)
      updateMeasurementStdDevs(distanceToTargetM);

      // Target yaw/pitch from best tracked target
      try {
        if (currentTagID != -1) {
          Cameras bestCam = vision.getbestCamera(currentTagID);
          bestCameraName = (bestCam != null) ? bestCam.name() : "NONE";
          if (bestCam != null) {
            var target = vision.getTargetFromId(currentTagID, bestCam);
            if (target != null) {
              targetYawDeg = sanitize(target.getYaw());
              targetPitchDeg = sanitize(target.getPitch());
            }
          }
        } else {
          targetYawDeg = 0;
          targetPitchDeg = 0;
          bestCameraName = "NONE";
        }
      } catch (Throwable t) {
        targetYawDeg = 0;
        targetPitchDeg = 0;
        bestCameraName = "NONE";
      }

      // Camera agreement: how many cameras see the current target
      try {
        int count = 0;
        if (currentTagID != -1) {
          if (vision.getTargetFromId(currentTagID, Cameras.LEFT_CAM) != null) count++;
          if (vision.getTargetFromId(currentTagID, Cameras.RIGHT_CAM) != null) count++;
        }
        camerasWithTarget = count;
      } catch (Throwable t) {
        camerasWithTarget = 0;
      }

      // Pipeline latency
      try {
        double lat = vision.getBestTargetLatencyMs();
        latencyMs = (lat >= 0) ? sanitize(lat) : 0;
      } catch (Throwable t) {
        latencyMs = 0;
      }

      // Per-camera connected status
      try {
        leftCamConnected = vision.isCameraConnected(Cameras.LEFT_CAM);
        rightCamConnected = vision.isCameraConnected(Cameras.RIGHT_CAM);
      } catch (Throwable t) {
        leftCamConnected = false;
        rightCamConnected = false;
      }

      visionHealthy = true;
    } catch (Throwable t) {
      visionHealthy = false;
      setDefaultValues();
    }
  }

  private void setDefaultValues() {
    hasTarget = false;
    lockedOnTarget = false;
    stableLock = false;
    lockedTagID = -1;
    consecutiveFrames = 0;
    sameTagFrameCount = 0;
    distanceToTargetM = 0;
    poseConfidence = 0;
    lockDurationMs = 0;
    poseTimestampSec = 0;
    measurementStdDevs = new double[] {0.5, 0.5, 0.5};
    targetYawDeg = 0;
    targetPitchDeg = 0;
    latencyMs = 0;
    leftCamConnected = false;
    rightCamConnected = false;
    bestCameraName = "NONE";
    camerasWithTarget = 0;
  }

  private void updateMeasurementStdDevs(double distanceM) {
    // Scale std devs with distance squared (photogrammetry principle)
    // At 1m: low uncertainty, at 4m+: high uncertainty
    double scale = Math.max(0.5, distanceM * distanceM * 0.1);
    measurementStdDevs[0] = Math.min(2.0, 0.1 * scale); // x (meters)
    measurementStdDevs[1] = Math.min(2.0, 0.1 * scale); // y (meters)
    measurementStdDevs[2] = Math.min(1.0, 0.05 * scale); // theta (radians)
  }

  private double computeConfidence(double distance, int frameCount, boolean stable) {
    double score = 100;

    // Penalize long distance (less accurate)
    if (distance > 5.0) score -= 30;
    else if (distance > 3.0) score -= 15;
    else if (distance > 2.0) score -= 5;

    // Penalize few frames (less reliable)
    if (frameCount < 3) score -= 30;
    else if (frameCount < 5) score -= 15;
    else if (frameCount < 10) score -= 5;

    // Bonus for stable lock
    if (stable) score += 10;

    return Math.max(0, Math.min(100, score));
  }

  @Override
  public void log() {
    SafeLog.put("Vision/Available", subsystemAvailable);
    SafeLog.put("Vision/Healthy", visionHealthy);
    SafeLog.put("Vision/HasTarget", hasTarget);
    SafeLog.put("Vision/LockedOnTarget", lockedOnTarget);
    SafeLog.put("Vision/ConsecutiveFrames", consecutiveFrames);
    SafeLog.put("Vision/TimeSinceLastTargetMs", timeSinceLastTargetMs);
    SafeLog.put("Vision/LockedTagID", lockedTagID);
    SafeLog.put("Vision/SameTagFrames", sameTagFrameCount);
    SafeLog.put("Vision/StableLock", stableLock);

    // Quality metrics
    SafeLog.put("Vision/Quality/DistanceM", distanceToTargetM);
    SafeLog.put("Vision/Quality/Confidence", poseConfidence);
    SafeLog.put("Vision/Quality/LockDurationMs", lockDurationMs);

    // Pose timing and trust weights
    SafeLog.put("Vision/PoseTimestampSec", poseTimestampSec);
    SafeLog.put("Vision/MeasurementStdDevs", measurementStdDevs);

    // Target orientation and pipeline health
    SafeLog.put("Vision/TargetYawDeg", targetYawDeg);
    SafeLog.put("Vision/TargetPitchDeg", targetPitchDeg);
    SafeLog.put("Vision/LatencyMs", latencyMs);
    SafeLog.put("Vision/Camera/LeftCam/Connected", leftCamConnected);
    SafeLog.put("Vision/Camera/RightCam/Connected", rightCamConnected);

    SafeLog.put("Vision/BestCamera", bestCameraName);
    SafeLog.put("Vision/CamerasWithTarget", camerasWithTarget);
  }

  @Override
  public String getName() {
    return "Vision";
  }

  // Accessors for TelemetryManager
  public boolean isLockedOnTarget() {
    return lockedOnTarget;
  }

  public boolean hasTarget() {
    return hasTarget;
  }
}
