package frc.robot.sim;

/** Simulation scenario interface for SimGUI testing. */
public interface SimScenario {
  /** Called once when scenario starts */
  void init();

  /** Called every robot cycle (~20ms). timestampSec is FPGA time. */
  void execute(double timestampSec);

  /** Returns true when scenario is complete */
  boolean isFinished();

  /** Scenario name for dashboard selection */
  String getName();
}
