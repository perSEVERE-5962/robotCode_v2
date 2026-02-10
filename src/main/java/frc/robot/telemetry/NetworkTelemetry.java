package frc.robot.telemetry;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

/** Network bandwidth monitoring for R704 compliance (VH-109: 7 Mbps). */
public class NetworkTelemetry implements SubsystemTelemetry {
  // R704 limits in Mbps (USA = VH-109)
  private static final double VH109_LIMIT_MBPS = 7.0;
  private static final double WARNING_THRESHOLD = 0.70;
  private static final double CRITICAL_THRESHOLD = 0.90;

  // Bandwidth estimation
  private double bandwidthLimitMbps = VH109_LIMIT_MBPS;
  private double estimatedBandwidthMbps = 0.0;
  private double bandwidthPercent = 0.0;
  private boolean bandwidthWarning = false;
  private boolean bandwidthCritical = false;

  // NT traffic tracking
  private final NetworkTableInstance ntInstance;
  // WPILib has no byte-counting API; this is estimated
  private double smoothedBandwidthMbps = 0.0;

  public NetworkTelemetry() {
    ntInstance = NetworkTableInstance.getDefault();
  }

  @Override
  public void update() {
    long currentTimeNs = System.nanoTime();

    // Estimate bandwidth from various sources
    estimatedBandwidthMbps = estimateBandwidth(currentTimeNs);

    // Exponential smoothing for stability
    if (smoothedBandwidthMbps == 0) {
      smoothedBandwidthMbps = estimatedBandwidthMbps;
    } else {
      smoothedBandwidthMbps = 0.9 * smoothedBandwidthMbps + 0.1 * estimatedBandwidthMbps;
    }

    // L3: Guard against division by zero or invalid limit
    double limit = bandwidthLimitMbps;
    if (!Double.isFinite(limit) || limit <= 0) {
      limit = VH109_LIMIT_MBPS;
    }
    bandwidthPercent = (smoothedBandwidthMbps / limit) * 100.0;
    if (!Double.isFinite(bandwidthPercent)) {
      bandwidthPercent = 0;
    }

    bandwidthWarning = bandwidthPercent > (WARNING_THRESHOLD * 100);
    bandwidthCritical = bandwidthPercent > (CRITICAL_THRESHOLD * 100);
  }

  private double estimateBandwidth(long currentTimeNs) {
    if (!DriverStation.isDSAttached()) {
      return 0.0;
    }

    // Base estimates for typical FRC traffic
    // Driver Station control: ~0.05 Mbps
    // NetworkTables (~126 signals @ 50Hz): ~0.30 Mbps
    // Protocol overhead: ~0.05 Mbps
    double baseTrafficMbps = 0.40;

    // Camera stream estimate (if streaming)
    // Typical: 640x480 @ 15fps compressed = ~1.5 Mbps
    double cameraEstimateMbps = 1.5;

    // TODO: If camera stream detection is available, adjust estimate
    // For now, assume camera is streaming during enabled modes
    double totalEstimate = baseTrafficMbps;
    if (DriverStation.isEnabled()) {
      totalEstimate += cameraEstimateMbps;
    }

    return totalEstimate;
  }

  @Override
  public void log() {
    SafeLog.put("Network/BandwidthMbps", smoothedBandwidthMbps);
    SafeLog.put("Network/BandwidthPercent", bandwidthPercent);
    SafeLog.put("Network/BandwidthWarning", bandwidthWarning);
    SafeLog.put("Network/BandwidthCritical", bandwidthCritical);
    SafeLog.put("Network/RadioType", "VH-109");
    SafeLog.put("Network/BandwidthLimitMbps", bandwidthLimitMbps);
  }

  @Override
  public String getName() {
    return "Network";
  }

  // Accessors for AlertManager
  public double getBandwidthPercent() {
    return bandwidthPercent;
  }

  public boolean isWarning() {
    return bandwidthWarning;
  }

  public boolean isCritical() {
    return bandwidthCritical;
  }
}
