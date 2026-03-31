package frc.robot.subsystems;

public interface Actuator {
  public double getPosition();

  public double getMotorVelocity();

  public void moveToPositionWithPID(double position);

  public void moveToVelocityWithPID(double rpm);

  public void move(double speed);

  public int getStickyFaultsRaw();

  public void updatePID(double kP, double kI, double kD, double kF);
}
