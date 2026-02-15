package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.MethodName.class)
class CommandsTelemetryTest extends TelemetryTestBase {

  private CommandsTelemetry telemetry;

  @BeforeEach
  void setUp() {
    // Commands need enabled DS to initialize
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
  void testGetNameReturnsCommands() {
    assertEquals("Commands", telemetry.getName());
  }

  @Test
  void testUpdateDoesNotThrow() {
    assertDoesNotThrow(
        () -> {
          telemetry.update();
          telemetry.log();
        });
  }

  @Test
  void testLogDoesNotThrow() {
    telemetry.update();
    assertDoesNotThrow(() -> telemetry.log());
  }

  @Test
  void testDefaultActiveListNone() throws Exception {
    telemetry.update();
    String list = getField(telemetry, "activeList");
    assertEquals("none", list);
  }

  @Test
  void testDefaultActiveCountZero() throws Exception {
    telemetry.update();
    int count = getField(telemetry, "activeCount");
    assertEquals(0, count);
  }

  @Test
  void testDefaultTotalExecutionsZero() {
    assertEquals(0, telemetry.getTotalExecutions());
  }

  @Test
  void testDefaultLastDurationZero() {
    assertEquals(0, telemetry.getLastCommandDurationMs(), 0.01);
  }

  @Test
  void testCommandInitializeIncrementsExecutions() {
    Command cmd = new InstantCommand(() -> {}).withName("TestCmd");
    cmd.schedule();
    CommandScheduler.getInstance().run();

    assertTrue(telemetry.getTotalExecutions() > 0, "Execution count should increment");
  }

  @Test
  void testCommandFinishTracked() throws Exception {
    Command cmd = new InstantCommand(() -> {}).withName("QuickCmd");
    cmd.schedule();
    CommandScheduler.getInstance().run();
    // InstantCommand finishes immediately after one run()

    int finished = getField(telemetry, "finishedCount");
    assertTrue(finished > 0, "Finished count should increment");
  }

  @Test
  void testActiveListUpdatedWithRunningCommand() throws Exception {
    Command cmd = new WaitCommand(10).withName("LongRunning");
    cmd.schedule();
    CommandScheduler.getInstance().run();
    telemetry.update();

    String list = getField(telemetry, "activeList");
    assertTrue(list.contains("LongRunning"), "Active list should contain running command name");

    cmd.cancel();
    CommandScheduler.getInstance().run();
  }

  @Test
  void testActiveCountMatchesRunningCommands() throws Exception {
    Command cmd1 = new WaitCommand(10).withName("Cmd1");
    Command cmd2 = new WaitCommand(10).withName("Cmd2");
    cmd1.schedule();
    cmd2.schedule();
    CommandScheduler.getInstance().run();
    telemetry.update();

    int count = getField(telemetry, "activeCount");
    assertEquals(2, count, "Should track 2 active commands");

    cmd1.cancel();
    cmd2.cancel();
    CommandScheduler.getInstance().run();
  }

  @Test
  void testInterruptedCountTracked() throws Exception {
    Command cmd = new WaitCommand(10).withName("InterruptMe");
    cmd.schedule();
    CommandScheduler.getInstance().run();

    cmd.cancel();
    CommandScheduler.getInstance().run();

    int interrupted = getField(telemetry, "interruptedCount");
    assertTrue(interrupted > 0, "Interrupted count should increment on cancel");
  }

  @Test
  void testActiveCountDecrementsOnFinish() throws Exception {
    Command cmd = new InstantCommand(() -> {}).withName("Quick");
    cmd.schedule();
    CommandScheduler.getInstance().run();
    telemetry.update();

    int count = getField(telemetry, "activeCount");
    assertEquals(0, count, "Active count should be 0 after instant command finishes");
  }
}
