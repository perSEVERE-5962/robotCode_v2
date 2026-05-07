package frc.robot.sim;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;

/**
 * Deterministic Xbox controller input playback for simulation. Wraps XboxControllerSim(0) to match
 * RobotContainer's driverXbox port.
 *
 * <p>Drive axes go through SimDriveOverride (bypasses HAL race with SimGUI). Buttons still go
 * through XboxControllerSim (triggers need HAL path).
 *
 * <p>Button mapping (from RobotContainer.configureBindings): A (whileTrue) ->
 * MoveShooter(DESIRED_SHOOTER_RPM) Y (whileTrue) -> MoveIndexer(DESIRED_INDEXER_RPM) X (onTrue) ->
 * SpeedUpThenIndex LeftStick -> Swerve drive (field-oriented angular velocity) Start -> Zero gyro
 * LBumper -> Wheel lock
 */
public class SimInputPlayback {
  private final XboxControllerSim xbox;
  private final XboxControllerSim copilot;

  public SimInputPlayback() {
    xbox = new XboxControllerSim(0);
    copilot = new XboxControllerSim(1);
  }

  public void driveForward(double magnitude) {
    SimDriveOverride.setY(magnitude);
  }

  public void strafe(double magnitude) {
    SimDriveOverride.setX(magnitude);
  }

  public void rotate(double magnitude) {
    SimDriveOverride.setOmega(magnitude);
  }

  /** Hold A to spin shooter (MoveShooter command) */
  public void holdA(boolean pressed) {
    xbox.setAButton(pressed);
    notifyDS();
  }

  /** Hold Y to run indexer (MoveIndexer command) */
  public void holdY(boolean pressed) {
    xbox.setYButton(pressed);
    notifyDS();
  }

  /** Tap X for SpeedUpThenIndex (onTrue trigger) */
  public void tapX() {
    xbox.setXButton(true);
    notifyDS();
  }

  public void releaseX() {
    xbox.setXButton(false);
    notifyDS();
  }

  /** Press Start to zero gyro */
  public void pressStart() {
    xbox.setStartButton(true);
    notifyDS();
  }

  public void releaseStart() {
    xbox.setStartButton(false);
    notifyDS();
  }

  /** Hold left bumper for wheel lock */
  public void holdLeftBumper(boolean pressed) {
    xbox.setLeftBumperButton(pressed);
    notifyDS();
  }

  /** Hold B for RunIntake (deploy intake pivot + run intake motor) */
  public void holdB(boolean pressed) {
    xbox.setBButton(pressed);
    notifyDS();
  }

  public void holdRightBumper(boolean pressed) {
    xbox.setRightBumperButton(pressed);
    notifyDS();
  }

  public void setLeftTrigger(double value) {
    xbox.setLeftTriggerAxis(value);
    notifyDS();
  }

  public void setRightTrigger(double value) {
    xbox.setRightTriggerAxis(value);
    notifyDS();
  }

  // === Copilot (port 1) methods ===

  /** Copilot A: DeployIntake + HoldAndIntake (whileTrue) */
  public void copilotHoldA(boolean pressed) {
    copilot.setAButton(pressed);
    notifyDS();
  }

  /** Copilot B: AgitateAndIndex (whileTrue) */
  public void copilotHoldB(boolean pressed) {
    copilot.setBButton(pressed);
    notifyDS();
  }

  /** Copilot X: MoveIntake (whileTrue) */
  public void copilotHoldX(boolean pressed) {
    copilot.setXButton(pressed);
    notifyDS();
  }

  /** Copilot Right Trigger: SpeedUpThenIndex (whileTrue) */
  public void copilotSetRightTrigger(double value) {
    copilot.setRightTriggerAxis(value);
    notifyDS();
  }

  /** Copilot Right Bumper: PivotIntake down (whileTrue) */
  public void copilotHoldRightBumper(boolean pressed) {
    copilot.setRightBumperButton(pressed);
    notifyDS();
  }

  /** Copilot Left Bumper: PivotIntake up (whileTrue) */
  public void copilotHoldLeftBumper(boolean pressed) {
    copilot.setLeftBumperButton(pressed);
    notifyDS();
  }

  public void releaseAll() {
    SimDriveOverride.reset();
    xbox.setAButton(false);
    xbox.setYButton(false);
    xbox.setXButton(false);
    xbox.setBButton(false);
    xbox.setStartButton(false);
    xbox.setBackButton(false);
    xbox.setLeftBumperButton(false);
    xbox.setRightBumperButton(false);
    xbox.setLeftTriggerAxis(0);
    xbox.setRightTriggerAxis(0);
    copilot.setAButton(false);
    copilot.setBButton(false);
    copilot.setXButton(false);
    copilot.setLeftBumperButton(false);
    copilot.setRightBumperButton(false);
    copilot.setLeftTriggerAxis(0);
    copilot.setRightTriggerAxis(0);
    notifyDS();
  }

  private void notifyDS() {
    DriverStationSim.notifyNewData();
  }
}
