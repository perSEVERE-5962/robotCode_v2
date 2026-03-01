package frc.robot.sim;

/**
 * Static volatile fields for sim drive input,bypasses HAL joystick port 0 which halsim_gui
 * continuously zeros. RobotContainer reads these via method references as SwerveInputStream
 * suppliers in sim mode.
 */
public final class SimDriveOverride {

  private static volatile double y = 0;
  private static volatile double x = 0;
  private static volatile double omega = 0;

  private SimDriveOverride() {}

  public static double getY() {
    return y;
  }

  public static double getX() {
    return x;
  }

  public static double getOmega() {
    return omega;
  }

  public static void setY(double value) {
    y = value;
  }

  public static void setX(double value) {
    x = value;
  }

  public static void setOmega(double value) {
    omega = value;
  }

  public static void reset() {
    y = 0;
    x = 0;
    omega = 0;
  }
}
