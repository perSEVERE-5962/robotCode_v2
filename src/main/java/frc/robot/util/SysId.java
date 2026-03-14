package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeActuator;
import frc.robot.subsystems.Shooter;

public class SysId {
  private static final int delay = 5; // seconds

  public static Command agitatorSysIdCommand() {
    Agitator inst = Agitator.getInstance();
    SysIdRoutine routine = new SysIdRoutine(
      new Config(null, null, null, (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new Mechanism((voltage) -> inst.getMotor().setVoltage(voltage), null, inst));
    return routine.quasistatic(Direction.kForward).andThen(Commands.waitSeconds(delay))
        .andThen(routine.quasistatic(Direction.kReverse)).andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kForward)).andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kReverse)).andThen(Commands.waitSeconds(delay));
  }

  public static Command hangerSysIdCommand() {
    Hanger inst = Hanger.getInstance();
    SysIdRoutine routine = new SysIdRoutine(
      new Config(null, null, null, (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new Mechanism((voltage) -> inst.getMotor().setVoltage(voltage), null, inst));
    return routine.quasistatic(Direction.kForward).andThen(Commands.waitSeconds(delay))
        .andThen(routine.quasistatic(Direction.kReverse)).andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kForward)).andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kReverse)).andThen(Commands.waitSeconds(delay));
  }

  public static Command indexerSysIdCommand() {
    Indexer inst = Indexer.getInstance();
    SysIdRoutine routine = new SysIdRoutine(
      new Config(null, null, null, (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new Mechanism((voltage) -> inst.getMotor().setVoltage(voltage), null, inst));
    return routine.quasistatic(Direction.kForward).andThen(Commands.waitSeconds(delay))
        .andThen(routine.quasistatic(Direction.kReverse)).andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kForward)).andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kReverse)).andThen(Commands.waitSeconds(delay));
  }

  public static Command intakeRollerSysIdCommand() {
    Intake inst = Intake.getInstance();
    SysIdRoutine routine = new SysIdRoutine(
      new Config(null, null, null, (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new Mechanism((voltage) -> inst.getMotor().setVoltage(voltage), null, inst));
    return routine.quasistatic(Direction.kForward).andThen(Commands.waitSeconds(delay))
        .andThen(routine.quasistatic(Direction.kReverse)).andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kForward)).andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kReverse)).andThen(Commands.waitSeconds(delay));
  }

  public static Command intakePivotSysIdCommand() {
    IntakeActuator inst = IntakeActuator.getInstance();
    SysIdRoutine routine = new SysIdRoutine(
      new Config(null, null, null, (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new Mechanism((voltage) -> inst.getMotor().setVoltage(voltage), null, inst));
    return routine.quasistatic(Direction.kForward).andThen(Commands.waitSeconds(delay))
        .andThen(routine.quasistatic(Direction.kReverse)).andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kForward)).andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kReverse)).andThen(Commands.waitSeconds(delay));
  }

  public static Command shooterSysIdCommand() {
    Shooter inst = Shooter.getInstance();
    SysIdRoutine routine = new SysIdRoutine(
      new Config(null, null, null, (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new Mechanism((voltage) -> inst.getMotor().setVoltage(voltage), null, inst));
    return routine.quasistatic(Direction.kForward).andThen(Commands.waitSeconds(delay))
        .andThen(routine.quasistatic(Direction.kReverse)).andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kForward)).andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kReverse)).andThen(Commands.waitSeconds(delay));
  }
}
