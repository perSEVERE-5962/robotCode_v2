package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class BallVision extends SubsystemBase {

    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    private List<PhotonTrackedTarget> fuelTargets = new ArrayList<>();

    public BallVision(String cameraName) {
        camera = new PhotonCamera(cameraName);
        camera.setPipelineIndex(VisionConstants.OBJECT_DETECT_PIPELINE);
    }

    @Override
    public void periodic() {
        latestResult = camera.getLatestResult();
        fuelTargets = filterFuelTargets(latestResult);

        SmartDashboard.putBoolean("Ball Detected", hasBall());
        SmartDashboard.putNumber("Ball Count", getBallCount());
        SmartDashboard.putBoolean("Cluster Visible", hasCluster());

        if (hasBall()) {
            SmartDashboard.putNumber("Best Ball Yaw", getBestBallYaw());
            SmartDashboard.putNumber("Best Ball Confidence",
                getBestBall().getDetectedObjectConfidence());
            SmartDashboard.putNumber("Cluster Yaw", getClusterYaw());
        }
    }

    private List<PhotonTrackedTarget> filterFuelTargets(PhotonPipelineResult result) {
        List<PhotonTrackedTarget> filtered = new ArrayList<>();
        if (result == null || !result.hasTargets()) return filtered;

        for (PhotonTrackedTarget target : result.getTargets()) {
            boolean correctClass =
                target.getDetectedObjectClassID() == VisionConstants.FUEL_CLASS_ID;
            boolean confidentEnough =
                target.getDetectedObjectConfidence() >= VisionConstants.MIN_CONFIDENCE;

            if (correctClass && confidentEnough) {
                filtered.add(target);
            }
        }
        return filtered;
    }

    public boolean hasBall() {
        return !fuelTargets.isEmpty();
    }

    public boolean hasCluster() {
        return fuelTargets.size() >= VisionConstants.CLUSTER_MIN_SIZE;
    }

    public int getBallCount() {
        return fuelTargets.size();
    }

    public PhotonTrackedTarget getBestBall() {
        return hasBall() ? fuelTargets.get(0) : null;
    }

    public double getBestBallYaw() {
        PhotonTrackedTarget t = getBestBall();
        return (t != null) ? t.getYaw() : 0.0;
    }

    public double getBestBallArea() {
        PhotonTrackedTarget t = getBestBall();
        return (t != null) ? t.getArea() : 0.0;
    }

    public double getClusterYaw() {
        if (!hasBall()) return 0.0;
        double totalYaw = 0.0;
        for (PhotonTrackedTarget t : fuelTargets) {
            totalYaw += t.getYaw();
        }
        return totalYaw / fuelTargets.size();
    }

    public double getClusterTotalArea() {
        double total = 0.0;
        for (PhotonTrackedTarget t : fuelTargets) {
            total += t.getArea();
        }
        return total;
    }

    public double getAverageConfidence() {
        if (!hasBall()) return 0.0;
        double total = 0.0;
        for (PhotonTrackedTarget t : fuelTargets) {
            total += t.getDetectedObjectConfidence();
        }
        return total / fuelTargets.size();
    }
}

