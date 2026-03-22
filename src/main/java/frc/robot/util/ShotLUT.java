package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/** Distance-to-ShotParameters LUT with interpolation. Bundles RPM, angle, and TOF together. */
public class ShotLUT {

  private final InterpolatingTreeMap<Double, ShotParameters> map;
  private int entryCount = 0;

  public ShotLUT() {
    map =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotParameters.interpolator());
  }

  /** Add or overwrite params at a distance. */
  public void put(double distanceM, ShotParameters params) {
    map.put(distanceM, params);
    entryCount++;
  }

  /** Shorthand for individual fields. */
  public void put(double distanceM, double rpm, double angleDeg, double tofSec) {
    put(distanceM, new ShotParameters(rpm, angleDeg, tofSec));
  }

  /** Interpolated params at a distance, or ZERO if the map is empty. */
  public ShotParameters get(double distanceM) {
    ShotParameters result = map.get(distanceM);
    return result != null ? result : ShotParameters.ZERO;
  }

  /** RPM at distance. */
  public double getRPM(double distanceM) {
    return get(distanceM).rpm();
  }

  /** Hood angle at distance. */
  public double getAngle(double distanceM) {
    return get(distanceM).angleDeg();
  }

  /** TOF at distance. */
  public double getTOF(double distanceM) {
    return get(distanceM).tofSec();
  }

  /** Remove all entries. */
  public void clear() {
    map.clear();
    entryCount = 0;
  }

  /** Rough entry count (overcounts if you put the same distance twice). */
  public int size() {
    return entryCount;
  }
}
