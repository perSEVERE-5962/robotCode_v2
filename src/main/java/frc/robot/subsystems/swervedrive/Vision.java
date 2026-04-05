package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.util.VisionFilter;
import frc.robot.util.VisionFilter.RejectionReason;
import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class Vision {

  /** April Tag Field Layout of the year. */
  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  /** Photon Vision Simulation */
  public VisionSystemSim visionSim;

  /** Current pose from the pose estimator using wheel odometry. */
  private Supplier<Pose2d> currentPose;

  /** Field from {@link swervelib.SwerveDrive#field} */
  private Field2d field2d;

  // Target lock tracking (for ReadyToShoot composite)
  private static final int LOCK_THRESHOLD_FRAMES = 5;
  private int consecutiveTargetFrames = 0;
  private double lastTargetTimestamp = 0;
  private boolean lastHasTarget = false;

  // Vision filtering stats (read by VisionTelemetry)
  private double autoStartTimestamp = 0;
  private int acceptedCount = 0;
  private int rejectedCount = 0;
  private int[] rejectionsByGate = new int[RejectionReason.values().length];
  private RejectionReason lastRejection = RejectionReason.ACCEPTED;
  private boolean blendingActive = false;
  private double blendWeight = 0;
  private boolean manualOverride = false;

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
   * @param field Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field) {
    this.currentPose = currentPose;
    this.field2d = field;

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (Cameras c : Cameras.values()) {
        c.addToVisionSim(visionSim);
      }

      openSimCameraViews();
    }
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the
   *     robot to position itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException(
          "Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }
  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    if (SwerveDriveTelemetry.isSimulation
        && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
      /*
       * In the maple-sim, odometry is simulated using encoder values, accounting for
       * factors like skidding and drifting.
       * As a result, the odometry may not always be 100% accurate.
       * However, the vision system should be able to provide a reasonably accurate
       * pose estimation, even when odometry is incorrect.
       * (This is why teams implement vision system to correct odometry.)
       * Therefore, we must ensure that the actual robot pose is provided in the
       * simulator when updating the vision simulation during the simulation.
       */
      visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
    }

    // Track auto start for pose jump grace period
    if (DriverStation.isAutonomousEnabled() && autoStartTimestamp == 0) {
      autoStartTimestamp = Timer.getFPGATimestamp();
    } else if (!DriverStation.isAutonomousEnabled()) {
      autoStartTimestamp = 0;
    }

    // Skip POSE_JUMP gate until we've accepted at least one pose, otherwise
    // the first vision correction after boot gets rejected and vision locks out forever
    double autoElapsed =
        (acceptedCount == 0)
            ? 0
            : (autoStartTimestamp > 0) ? Timer.getFPGATimestamp() - autoStartTimestamp : 999;

    if (manualOverride) {
      return;
    }

    Pose2d currentFusedPose = swerveDrive.getPose();
    Rotation2d gyroHeading = swerveDrive.getOdometryHeading();
    ChassisSpeeds fieldVel = swerveDrive.getFieldVelocity();
    double speedMps = Math.hypot(fieldVel.vxMetersPerSecond, fieldVel.vyMetersPerSecond);

    blendingActive = false;
    blendWeight = 0;

    double now = Timer.getFPGATimestamp();

    for (Cameras camera : Cameras.values()) {
      camera.poseEstimator.addHeadingData(
          Timer.getFPGATimestamp(), swerveDrive.getPose().getRotation());
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isPresent()) {
        var pose = poseEst.get();
        // swerveDrive.addVisionMeasurement(
        //    pose.estimatedPose.toPose2d(), pose.timestampSeconds, camera.curStdDevs);

        // Reject stale (>1s old) or future timestamps
        double age = now - pose.timestampSeconds;
        if (age < 0 || age > 1.0) {
          continue;
        }

        int tagCount = pose.targetsUsed.size();
        double worstAmbiguity = getWorstAmbiguity(pose);

        RejectionReason reason =
            VisionFilter.evaluate(
                pose.estimatedPose,
                tagCount,
                worstAmbiguity,
                gyroHeading,
                currentFusedPose,
                autoElapsed);

        if (reason != RejectionReason.ACCEPTED) {
          rejectedCount++;
          rejectionsByGate[reason.ordinal()]++;
          lastRejection = reason;
          continue;
        }

        acceptedCount++;

        double avgDist = getAverageTagDistance(pose, swerveDrive);
        Matrix<N3, N1> stdDevs =
            VisionFilter.computeStdDevs(
                tagCount,
                avgDist,
                speedMps,
                camera.getSingleTagStdDevs(),
                camera.getMultiTagStdDevs());

        // Pose blending for single-tag close estimates
        Pose2d poseToUse = pose.estimatedPose.toPose2d();
        if (tagCount == 1 && avgDist < VisionFilter.BLEND_DISTANCE_THRESHOLD_M) {
          double w = VisionFilter.computeBlendWeight(avgDist);
          if (w > 0) {
            poseToUse = VisionFilter.blendPose(currentFusedPose, poseToUse, w);
            blendingActive = true;
            blendWeight = Math.max(blendWeight, w);
          }
        }

        swerveDrive.addVisionMeasurement(poseToUse, pose.timestampSeconds, stdDevs);
      }
    }
  }

  /** Get the worst (highest) ambiguity across all targets in an estimate. */
  private double getWorstAmbiguity(EstimatedRobotPose est) {
    double worst = 0;
    for (var target : est.targetsUsed) {
      worst = Math.max(worst, target.getPoseAmbiguity());
    }
    return worst;
  }

  /** Get average distance from estimated pose to all visible tags. */
  private double getAverageTagDistance(EstimatedRobotPose est, SwerveDrive swerveDrive) {
    double totalDist = 0;
    int count = 0;
    for (var target : est.targetsUsed) {
      var tagPose = fieldLayout.getTagPose(target.getFiducialId());
      if (tagPose.isPresent()) {
        totalDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(est.estimatedPose.toPose2d().getTranslation());
        count++;
      }
    }
    return (count > 0) ? totalDist / count : 5.0;
  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   *
   * <ul>
   *   <li>No Pose Estimates could be generated
   *   <li>The generated pose estimate was considered not accurate
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to
   *     create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    if (Robot.isSimulation()) {
      Field2d debugField = visionSim.getDebugField();
      // Uncomment to enable outputting of vision targets in sim.
      poseEst.ifPresentOrElse(
          est -> debugField.getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
          () -> {
            debugField.getObject("VisionEstimation").setPoses();
          });
    }
    return poseEst;
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d()))
        .orElse(-1.0);
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return target;
  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim() {
    return visionSim;
  }

  /**
   * Open up the photon vision camera streams on the localhost, assumes running photon vision on
   * localhost.
   */
  private void openSimCameraViews() {
    if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
      // try
      // {
      // Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
      // Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
      // Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
      // } catch (IOException | URISyntaxException e)
      // {
      // e.printStackTrace();
      // }
    }
  }

  /** Update the {@link Field2d} to include tracked targets/ */
  public void updateVisionField() {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values()) {
      if (!c.resultsList.isEmpty()) {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets()) {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

  /** Camera Enum to select each camera */
  public enum Cameras {
    /** Left Camera */
    LEFT_CAM(
        "back-left",
        new Rotation3d(0, Math.toRadians(-15), Math.toRadians(135)),
        new Translation3d(-0.293, 0.293, 0.229),
        VecBuilder.fill(0.3, 0.3, 0.6),
        VecBuilder.fill(0.1, 0.1, 0.2)),

    /** Right Camera */
    RIGHT_CAM(
        "back-right",
        new Rotation3d(0, Math.toRadians(-15), Math.toRadians(-135)),
        new Translation3d(-0.293, -0.293, 0.229),
        VecBuilder.fill(0.3, 0.3, 0.6),
        VecBuilder.fill(0.1, 0.1, 0.2)),

    /** Front-left camera, angled 45 deg outward */
    FRONT_LEFT_CAM(
        "front-left",
        new Rotation3d(0, Math.toRadians(0), Math.toRadians(35)),
        new Translation3d(
            Units.inchesToMeters(-4.44), Units.inchesToMeters(5.7), Units.inchesToMeters(19)),
        VecBuilder.fill(0.3, 0.3, 0.6),
        VecBuilder.fill(0.1, 0.1, 0.2)),

    /** Front-right camera, angled 45 deg outward (mirrored from front-left) */
    FRONT_RIGHT_CAM(
        "front-right",
        new Rotation3d(0, Math.toRadians(0), Math.toRadians(-35)),
        new Translation3d(
            Units.inchesToMeters(-4.44), Units.inchesToMeters(-5.7), Units.inchesToMeters(19)),
        VecBuilder.fill(0.3, 0.3, 0.6),
        VecBuilder.fill(0.1, 0.1, 0.2));

    /** Latency alert to use when high latency is detected. */
    public final Alert latencyAlert;

    /** Camera instance for comms. */
    public final PhotonCamera camera;

    /** Pose estimator for camera. */
    public final PhotonPoseEstimator poseEstimator;

    /** Standard Deviation for single tag readings for pose estimation. */
    private final Matrix<N3, N1> singleTagStdDevs;

    /** Standard deviation for multi-tag readings for pose estimation. */
    private final Matrix<N3, N1> multiTagStdDevs;

    /** Transform of the camera rotation and translation relative to the center of the robot */
    private final Transform3d robotToCamTransform;

    /** Current standard deviations used. */
    public Matrix<N3, N1> curStdDevs;

    /** Estimated robot pose. */
    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    /** Simulated camera instance which only exists during simulations. */
    public PhotonCameraSim cameraSim;

    /** Results list to be updated periodically and cached to avoid unnecessary queries. */
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();

    /**
     * Construct a Photon Camera class with help. Standard deviations are fake values, experiment
     * and determine estimation noise on an actual robot.
     *
     * @param name Name of the PhotonVision camera found in the PV UI.
     * @param robotToCamRotation {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
     * @param singleTagStdDevs Single AprilTag standard deviations of estimated poses from the
     *     camera.
     * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the
     *     camera.
     */
    Cameras(
        String name,
        Rotation3d robotToCamRotation,
        Translation3d robotToCamTranslation,
        Matrix<N3, N1> singleTagStdDevs,
        Matrix<N3, N1> multiTagStdDevsMatrix) {
      latencyAlert =
          new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      camera = new PhotonCamera(name);

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout, robotToCamTransform);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;

      if (Robot.isSimulation()) {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in
        // pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop
        // rate).
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
      }
    }

    /**
     * Add camera to {@link VisionSystemSim} for simulated photon vision.
     *
     * @param systemSim {@link VisionSystemSim} to use.
     */
    public void addToVisionSim(VisionSystemSim systemSim) {
      if (Robot.isSimulation()) {
        systemSim.addCamera(cameraSim, robotToCamTransform);
      }
    }

    public Matrix<N3, N1> getSingleTagStdDevs() {
      return singleTagStdDevs;
    }

    public Matrix<N3, N1> getMultiTagStdDevs() {
      return multiTagStdDevs;
    }

    /**
     * Get the result with the least ambiguity from the best tracked target within the Cache. This
     * may not be the most recent result!
     *
     * @return The result in the cache with the least ambiguous best tracked target. This is not the
     *     most recent result!
     */
    public Optional<PhotonPipelineResult> getBestResult() {
      if (resultsList.isEmpty()) {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult = resultsList.get(0);
      double ambiguity = bestResult.getBestTarget().getPoseAmbiguity();
      double currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList) {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < ambiguity && currentAmbiguity > 0) {
          bestResult = result;
          ambiguity = currentAmbiguity;
        }
      }
      return Optional.of(bestResult);
    }

    /**
     * Get the latest result from the current cache.
     *
     * @return Empty optional if nothing is found. Latest result if something is there.
     */
    public Optional<PhotonPipelineResult> getLatestResult() {
      return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    /**
     * Get the estimated robot pose. Updates the current robot pose estimation, standard deviations,
     * and flushes the cache of results.
     *
     * @return Estimated pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      updateUnreadResults();
      return estimatedRobotPose;
    }

    /**
     * Update the latest results, cached with a maximum refresh rate of 1req/15ms. Sorts the list by
     * timestamp.
     */
    private void updateUnreadResults() {
      double mostRecentTimestamp =
          resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();

      for (PhotonPipelineResult result : resultsList) {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }

      resultsList =
          Robot.isReal()
              ? camera.getAllUnreadResults()
              : cameraSim.getCamera().getAllUnreadResults();
      resultsList.sort(
          (PhotonPipelineResult a, PhotonPipelineResult b) -> {
            return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
          });
      if (!resultsList.isEmpty()) {
        updateEstimatedGlobalPose();
      }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved
     * with {@link Cameras#updateEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    private void updateEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList) {
        visionEst = poseEstimator.estimateCoprocMultiTagPose(change);
        if (visionEst.isEmpty()) {
          visionEst = poseEstimator.estimateLowestAmbiguityPose(change);
        }
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic
     * standard deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;

      } else {
        // Pose present. Start running Heuristic
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an
        // average-distance metric
        for (var tgt : targets) {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) {
            continue;
          }
          numTags++;
          avgDist +=
              tagPose
                  .get()
                  .toPose2d()
                  .getTranslation()
                  .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
          // No tags visible. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;
        } else {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) {
            estStdDevs = multiTagStdDevs;
          }
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }

    public Transform3d getRobotToCamera() {
      return robotToCamTransform;
    }
  }

  public Cameras getbestCamera(int id) {
    PhotonTrackedTarget bestTarget = null;
    Cameras bestCamEnum = null;

    for (Cameras c : Cameras.values()) {
      Optional<PhotonPipelineResult> optionalResult = c.getLatestResult();

      if (optionalResult.isPresent() && optionalResult.get().hasTargets()) {
        for (var target : optionalResult.get().getTargets()) {
          if (target.getFiducialId() == id) {
            if (bestTarget == null || target.getArea() > bestTarget.getArea()) {
              bestTarget = target;
              bestCamEnum = c;
            }
          }
        }
      }
    }
    return bestCamEnum; // can be null if no camera sees the target
  }

  /** Check if any camera currently sees a target. */
  public boolean hasTarget() {
    for (Cameras c : Cameras.values()) {
      Optional<PhotonPipelineResult> result = c.getLatestResult();
      if (result.isPresent() && result.get().hasTargets()) {
        return true;
      }
    }
    return false;
  }

  /**
   * Get the fiducial ID of the best visible target (largest area).
   *
   * @return Tag ID, or -1 if no target visible
   */
  public int getBestTargetId() {
    PhotonTrackedTarget bestTarget = null;
    for (Cameras c : Cameras.values()) {
      Optional<PhotonPipelineResult> result = c.getLatestResult();
      if (result.isPresent() && result.get().hasTargets()) {
        for (PhotonTrackedTarget target : result.get().getTargets()) {
          if (bestTarget == null || target.getArea() > bestTarget.getArea()) {
            bestTarget = target;
          }
        }
      }
    }
    return (bestTarget != null) ? bestTarget.getFiducialId() : -1;
  }

  /**
   * Check if vision is locked on target (stable tracking for ReadyToShoot). Requires consecutive
   * frames with targets to confirm lock.
   */
  public boolean isLockedOnTarget() {
    return consecutiveTargetFrames >= LOCK_THRESHOLD_FRAMES;
  }

  /** Update target lock tracking. Call this in periodic or after updatePoseEstimation. */
  public void updateTargetLock() {
    double now = Timer.getFPGATimestamp();
    boolean currentHasTarget = hasTarget();

    if (currentHasTarget) {
      consecutiveTargetFrames++;
      lastTargetTimestamp = now;
    } else {
      consecutiveTargetFrames = 0;
    }
    lastHasTarget = currentHasTarget;
  }

  /**
   * Get pipeline latency from the best available result with targets.
   *
   * @return Latency in ms, or -1 if no result available
   */
  public double getBestTargetLatencyMs() {
    for (Cameras c : Cameras.values()) {
      Optional<PhotonPipelineResult> result = c.getLatestResult();
      if (result.isPresent() && result.get().hasTargets()) {
        return result.get().metadata.getLatencyMillis();
      }
    }
    return -1;
  }

  /**
   * Check if a specific camera is connected.
   *
   * @param cam Camera to check
   * @return true if connected
   */
  public boolean isCameraConnected(Cameras cam) {
    return cam != null && cam.camera.isConnected();
  }

  // Filter stats for VisionTelemetry

  public int getAcceptedCount() {
    return acceptedCount;
  }

  public int getRejectedCount() {
    return rejectedCount;
  }

  public int[] getRejectionsByGate() {
    return rejectionsByGate;
  }

  public RejectionReason getLastRejection() {
    return lastRejection;
  }

  public boolean isBlendingActive() {
    return blendingActive;
  }

  public double getBlendWeight() {
    return blendWeight;
  }

  public boolean isManualOverride() {
    return manualOverride;
  }

  public void setManualOverride(boolean override) {
    this.manualOverride = override;
  }

  public double getAcceptRatePct() {
    int total = acceptedCount + rejectedCount;
    return (total > 0) ? (acceptedCount * 100.0 / total) : 100.0;
  }
}
