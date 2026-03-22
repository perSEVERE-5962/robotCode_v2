package frc.robot.util;

import edu.wpi.first.math.interpolation.Interpolator;

/** RPM, hood angle, and TOF for one distance. All three interpolate linearly for the LUT. */
public record ShotParameters(double rpm, double angleDeg, double tofSec) {

  /** All zeros, used when the map is empty. */
  public static final ShotParameters ZERO = new ShotParameters(0, 0, 0);

  /** Linear interpolator for InterpolatingTreeMap. */
  public static Interpolator<ShotParameters> interpolator() {
    return (start, end, t) ->
        new ShotParameters(
            start.rpm + t * (end.rpm - start.rpm),
            start.angleDeg + t * (end.angleDeg - start.angleDeg),
            start.tofSec + t * (end.tofSec - start.tofSec));
  }
}
