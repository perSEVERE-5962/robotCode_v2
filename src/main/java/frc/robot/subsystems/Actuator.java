package frc.robot.subsystems;

public interface Actuator {
  public double getPosition();

  public double getMotorVelocity();

  public void moveToPositionWithPID(double position);

  public void moveToVelocityWithPID(double rpm);

  public void move(double speed);

  public int getStickyFaultsRaw();

  /**
   * Sticky warnings as raw bits (REVLib warning word). TalonFX has no separate warning register, so
   * the Talon-backed implementation returns 0 and the decoder skips that path.
   */
  public default int getStickyWarningsRaw() {
    return 0;
  }

  public void updatePID(double kP, double kI, double kD, double kF);
}
