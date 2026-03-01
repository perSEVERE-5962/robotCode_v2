package frc.robot.telemetry;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Cameras;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.VisionFilter.RejectionReason;

/** Vision telemetry: target lock, tag ID stability, pose confidence. */
public class VisionTelemetry implements SubsystemTelemetry {
  private Vision vision;
  private boolean subsystemAvailable = false;
  private boolean visionHealthy = true;

  private static final int LOCK_THRESHOLD_FRAMES = 5;
  private static final int STABLE_LOCK_THRESHOLD_FRAMES = 10;

  private boolean hasTarget = false;
  private boolean lockedOnTarget = false;
  private int consecutiveFrames = 0;
  private double lastTargetTimestamp = 0;
  private double timeSinceLastTargetMs = 0;

  private int lastTagID = -1;
  private int sameTagFrameCount = 0;
  private int lockedTagID = -1;
  private boolean stableLock = false;

  private boolean wasLockedForCycle = false;

  private double distanceToTargetM = 0;
  private double poseConfidence = 0;
  private double lockDurationMs = 0;
  private double lockStartTime = 0;
  private boolean wasLocked = false;

  private double poseTimestampSec = 0;
  private double[] measurementStdDevs = {0.5, 0.5, 0.5}; // x, y, theta defaults

  private double targetYawDeg = 0;
  private double targetPitchDeg = 0;
  private double latencyMs = 0;
  private boolean leftCamConnected = false;
  private boolean rightCamConnected = false;
  private boolean frontLeftCamConnected = false;
  private boolean frontRightCamConnected = false;

  private String bestCameraName = "NONE";
  private int camerasWithTarget = 0;

  private int filterAccepted = 0;
  private int filterRejected = 0;
  private double filterAcceptRatePct = 100;
  private String lastRejectionReason = "NONE";
  private boolean blendingActive = false;
  private double blendWeight = 0;
  private boolean manualOverride = false;

  private int rejectAmbiguity = 0;
  private int rejectZHeight = 0;
  private int rejectRollPitch = 0;
  private int rejectFieldBounds = 0;
  private int rejectHeadingDivergence = 0;
  private int rejectPoseJump = 0;

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

      wasLockedForCycle = lockedOnTarget;

      int currentTagID = vision.getBestTargetId();

      if (currentTagID == lastTagID && currentTagID != -1) {
        sameTagFrameCount++;
      } else {
        sameTagFrameCount = (currentTagID != -1) ? 1 : 0;
      }
      lastTagID = currentTagID;

      if (lockedOnTarget && currentTagID != -1) {
        lockedTagID = currentTagID;
      } else if (!lockedOnTarget) {
        lockedTagID = -1;
      }
      stableLock = sameTagFrameCount >= STABLE_LOCK_THRESHOLD_FRAMES;

      if (currentTagID != -1) {
        distanceToTargetM = sanitize(vision.getDistanceFromAprilTag(currentTagID));
      } else {
        distanceToTargetM = 0;
      }

      poseConfidence = computeConfidence(distanceToTargetM, sameTagFrameCount, stableLock);

      if (lockedOnTarget && !wasLocked) {
        lockStartTime = now;
      }
      if (lockedOnTarget) {
        lockDurationMs = sanitize((now - lockStartTime) * 1000);
      } else {
        lockDurationMs = 0;
      }
      wasLocked = lockedOnTarget;

      poseTimestampSec = hasTarget ? lastTargetTimestamp : now;
      updateMeasurementStdDevs(distanceToTargetM);

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

      try {
        int count = 0;
        if (currentTagID != -1) {
          for (Cameras cam : Cameras.values()) {
            if (vision.getTargetFromId(currentTagID, cam) != null) count++;
          }
        }
        camerasWithTarget = count;
      } catch (Throwable t) {
        camerasWithTarget = 0;
      }

      try {
        double lat = vision.getBestTargetLatencyMs();
        latencyMs = (lat >= 0) ? sanitize(lat) : 0;
      } catch (Throwable t) {
        latencyMs = 0;
      }

      try {
        leftCamConnected = vision.isCameraConnected(Cameras.LEFT_CAM);
        rightCamConnected = vision.isCameraConnected(Cameras.RIGHT_CAM);
        frontLeftCamConnected = vision.isCameraConnected(Cameras.FRONT_LEFT_CAM);
        frontRightCamConnected = vision.isCameraConnected(Cameras.FRONT_RIGHT_CAM);
      } catch (Throwable t) {
        leftCamConnected = false;
        rightCamConnected = false;
        frontLeftCamConnected = false;
        frontRightCamConnected = false;
      }

      visionHealthy = true;
    } catch (Throwable t) {
      visionHealthy = false;
      setDefaultValues();
    }

    try {
      if (vision != null) {
        filterAccepted = vision.getAcceptedCount();
        filterRejected = vision.getRejectedCount();
        filterAcceptRatePct = vision.getAcceptRatePct();
        RejectionReason lr = vision.getLastRejection();
        lastRejectionReason = (lr != null) ? lr.name() : "NONE";
        blendingActive = vision.isBlendingActive();
        blendWeight = vision.getBlendWeight();
        manualOverride = vision.isManualOverride();

        int[] byGate = vision.getRejectionsByGate();
        rejectAmbiguity = byGate[RejectionReason.AMBIGUITY.ordinal()];
        rejectZHeight = byGate[RejectionReason.Z_HEIGHT.ordinal()];
        rejectRollPitch = byGate[RejectionReason.ROLL_PITCH.ordinal()];
        rejectFieldBounds = byGate[RejectionReason.FIELD_BOUNDS.ordinal()];
        rejectHeadingDivergence = byGate[RejectionReason.HEADING_DIVERGENCE.ordinal()];
        rejectPoseJump = byGate[RejectionReason.POSE_JUMP.ordinal()];
      }
    } catch (Throwable t) {
      // filter stats are non-critical
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
    frontLeftCamConnected = false;
    frontRightCamConnected = false;
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
    // No target at all = zero confidence
    if (frameCount == 0) return 0;

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

    SafeLog.put("Vision/Quality/DistanceM", distanceToTargetM);
    SafeLog.put("Vision/Quality/Confidence", poseConfidence);
    SafeLog.put("Vision/Quality/LockDurationMs", lockDurationMs);
    SafeLog.put("Vision/PoseTimestampSec", poseTimestampSec);
    SafeLog.put("Vision/MeasurementStdDevs", measurementStdDevs);
    SafeLog.put("Vision/TargetYawDeg", targetYawDeg);
    SafeLog.put("Vision/TargetPitchDeg", targetPitchDeg);
    SafeLog.put("Vision/LatencyMs", latencyMs);
    SafeLog.put("Vision/Camera/LeftCam/Connected", leftCamConnected);
    SafeLog.put("Vision/Camera/RightCam/Connected", rightCamConnected);
    SafeLog.put("Vision/Camera/FrontLeftCam/Connected", frontLeftCamConnected);
    SafeLog.put("Vision/Camera/FrontRightCam/Connected", frontRightCamConnected);

    SafeLog.put("Vision/BestCamera", bestCameraName);
    SafeLog.put("Vision/CamerasWithTarget", camerasWithTarget);

    SafeLog.put("Vision/Filter/AcceptedCount", filterAccepted);
    SafeLog.put("Vision/Filter/RejectedCount", filterRejected);
    SafeLog.put("Vision/Filter/AcceptRatePct", filterAcceptRatePct);
    SafeLog.put("Vision/Filter/LastRejection", lastRejectionReason);
    SafeLog.put("Vision/Filter/Reject/Ambiguity", rejectAmbiguity);
    SafeLog.put("Vision/Filter/Reject/ZHeight", rejectZHeight);
    SafeLog.put("Vision/Filter/Reject/RollPitch", rejectRollPitch);
    SafeLog.put("Vision/Filter/Reject/FieldBounds", rejectFieldBounds);
    SafeLog.put("Vision/Filter/Reject/HeadingDivergence", rejectHeadingDivergence);
    SafeLog.put("Vision/Filter/Reject/PoseJump", rejectPoseJump);
    SafeLog.put("Vision/Blending/Active", blendingActive);
    SafeLog.put("Vision/Blending/Weight", blendWeight);
    SafeLog.put("Vision/ManualOverride", manualOverride);
  }

  @Override
  public String getName() {
    return "Vision";
  }

  public boolean isLockedOnTarget() {
    return lockedOnTarget;
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  public boolean isLeftCamConnected() {
    return leftCamConnected;
  }

  public boolean isRightCamConnected() {
    return rightCamConnected;
  }

  public double getPoseConfidence() {
    return poseConfidence;
  }
}
