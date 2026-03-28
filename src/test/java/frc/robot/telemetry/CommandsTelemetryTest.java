package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class CommandsTelemetryTest extends TelemetryTestBase {

  private CommandsTelemetry telemetry;

  @BeforeEach
  void setUp() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().clearComposedCommands();
    telemetry = new CommandsTelemetry();
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().run();
  }

  @Test
  void testCommandInitializeIncrementsExecutions() {
    Command cmd = new InstantCommand(() -> {}).withName("TestCmd");
    CommandScheduler.getInstance().schedule(cmd);
    CommandScheduler.getInstance().run();

    assertTrue(telemetry.getTotalExecutions() > 0, "Execution count should increment");
  }

  @Test
  void testCommandFinishTracked() throws Exception {
    Command cmd = new InstantCommand(() -> {}).withName("QuickCmd");
    CommandScheduler.getInstance().schedule(cmd);
    CommandScheduler.getInstance().run();

    int finished = getField(telemetry, "finishedCount");
    assertTrue(finished > 0, "Finished count should increment");
  }

  @Test
  void testActiveListUpdatedWithRunningCommand() throws Exception {
    Command cmd = new WaitCommand(10).withName("LongRunning");
    CommandScheduler.getInstance().schedule(cmd);
    CommandScheduler.getInstance().run();
    telemetry.update();

    String list = getField(telemetry, "activeList");
    assertTrue(list.contains("LongRunning"), "Active list should contain running command name");

    CommandScheduler.getInstance().cancel(cmd);
    CommandScheduler.getInstance().run();
  }

  @Test
  void testActiveCountMatchesRunningCommands() throws Exception {
    Command cmd1 = new WaitCommand(10).withName("Cmd1");
    Command cmd2 = new WaitCommand(10).withName("Cmd2");
    CommandScheduler.getInstance().schedule(cmd1);
    CommandScheduler.getInstance().schedule(cmd2);
    CommandScheduler.getInstance().run();
    telemetry.update();

    int count = getField(telemetry, "activeCount");
    assertEquals(2, count, "Should track 2 active commands");

    CommandScheduler.getInstance().cancel(cmd1);
    CommandScheduler.getInstance().cancel(cmd2);
    CommandScheduler.getInstance().run();
  }

  @Test
  void testInterruptedCountTracked() throws Exception {
    Command cmd = new WaitCommand(10).withName("InterruptMe");
    CommandScheduler.getInstance().schedule(cmd);
    CommandScheduler.getInstance().run();

    CommandScheduler.getInstance().cancel(cmd);
    CommandScheduler.getInstance().run();

    int interrupted = getField(telemetry, "interruptedCount");
    assertTrue(interrupted > 0, "Interrupted count should increment on cancel");
  }
}
