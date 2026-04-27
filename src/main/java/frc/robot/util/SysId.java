package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Shooter;

public class SysId {
  private static final int delay = 2; // seconds

  public static Command agitatorSysIdCommand() {
    Agitator inst = Agitator.getInstance();
    final VoltageOut m_voltReq = new VoltageOut(0.0);
    SysIdRoutine routine =
        new SysIdRoutine(
            new Config(Volts.of(0.5).per(Seconds), Volts.of(3.0), Seconds.of(6.0)),
            new Mechanism(
                (Voltage voltage) ->
                    inst.getMotor().setControl(m_voltReq.withOutput(voltage.in(Volts))),
                (SysIdRoutineLog log) -> {
                  log.motor("agitator")
                      .voltage(inst.getMotor().getMotorVoltage().getValue())
                      .linearPosition(Meters.of(inst.getPosition()))
                      .linearVelocity(MetersPerSecond.of(inst.getMotorVelocity()));
                },
                inst));
    return routine
        .quasistatic(Direction.kForward)
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.quasistatic(Direction.kReverse))
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kForward))
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kReverse))
        .andThen(Commands.waitSeconds(delay));
  }

  public static Command hangerSysIdCommand() {
    Hanger inst = Hanger.getInstance();
    SysIdRoutine routine =
        new SysIdRoutine(
            new Config(Volts.of(0.5).per(Seconds), Volts.of(3.0), Seconds.of(6.0)),
            new Mechanism(
                (Voltage voltage) -> inst.getMotor().setVoltage(voltage),
                (SysIdRoutineLog log) -> {
                  log.motor("hanger")
                      .voltage(Volts.of(inst.getAppliedOutput() * inst.getBusVoltage()))
                      .linearPosition(Meters.of(inst.getPosition()))
                      .linearVelocity(MetersPerSecond.of(inst.getMotorVelocity()));
                },
                inst));
    return routine
        .quasistatic(Direction.kForward)
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.quasistatic(Direction.kReverse))
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kForward))
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kReverse))
        .andThen(Commands.waitSeconds(delay));
  }

  public static Command indexerSysIdCommand() {
    Indexer inst = Indexer.getInstance();
    SysIdRoutine routine =
        new SysIdRoutine(
            new Config(Volts.of(0.5).per(Seconds), Volts.of(3.0), Seconds.of(6.0)),
            new Mechanism(
                (Voltage voltage) -> inst.getMotor().setVoltage(voltage),
                (SysIdRoutineLog log) -> {
                  log.motor("indexer")
                      .voltage(Volts.of(inst.getAppliedOutput() * inst.getBusVoltage()))
                      .linearPosition(Meters.of(inst.getPosition()))
                      .linearVelocity(MetersPerSecond.of(inst.getMotorVelocity()));
                },
                inst));
    return routine
        .quasistatic(Direction.kForward)
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.quasistatic(Direction.kReverse))
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kForward))
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kReverse))
        .andThen(Commands.waitSeconds(delay));
  }

  public static Command intakeRollerSysIdCommand() {
    IntakeRoller inst = IntakeRoller.getInstance();
    SysIdRoutine routine =
        new SysIdRoutine(
            new Config(Volts.of(0.5).per(Seconds), Volts.of(3.0), Seconds.of(6.0)),
            new Mechanism(
                (Voltage voltage) -> inst.getMotor().setVoltage(voltage),
                (SysIdRoutineLog log) -> {
                  log.motor("intakeRoller")
                      .voltage(Volts.of(inst.getAppliedOutput() * inst.getBusVoltage()))
                      .linearPosition(Meters.of(inst.getPosition()))
                      .linearVelocity(MetersPerSecond.of(inst.getMotorVelocity()));
                },
                inst));
    return routine
        .quasistatic(Direction.kForward)
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.quasistatic(Direction.kReverse))
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kForward))
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kReverse))
        .andThen(Commands.waitSeconds(delay));
  }

  public static Command intakePivotSysIdCommand() {
    IntakePivot inst = IntakePivot.getInstance();
    final VoltageOut m_voltReq = new VoltageOut(0.0);
    SysIdRoutine routine =
        new SysIdRoutine(
            new Config(Volts.of(0.5).per(Seconds), Volts.of(3.0), Seconds.of(6.0)),
            new Mechanism(
                (Voltage voltage) ->
                    inst.getMotor().setControl(m_voltReq.withOutput(voltage.in(Volts))),
                (SysIdRoutineLog log) -> {
                  log.motor("intakePivot")
                      .voltage(inst.getMotor().getMotorVoltage().getValue())
                      .linearPosition(Meters.of(inst.getPosition()))
                      .linearVelocity(MetersPerSecond.of(inst.getMotorVelocity()));
                },
                inst));
    return routine
        .quasistatic(Direction.kForward)
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.quasistatic(Direction.kReverse))
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kForward))
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kReverse))
        .andThen(Commands.waitSeconds(delay));
  }

  public static Command shooterSysIdCommand() {
    Shooter inst = Shooter.getInstance();
    SysIdRoutine routine =
        new SysIdRoutine(
            new Config(Volts.of(0.5).per(Seconds), Volts.of(3.0), Seconds.of(6.0)),
            new Mechanism(
                (Voltage voltage) -> inst.getMotor().setVoltage(voltage),
                (SysIdRoutineLog log) -> {
                  log.motor("shooter")
                      .voltage(Volts.of(inst.getAppliedOutput() * inst.getBusVoltage()))
                      .linearPosition(Meters.of(inst.getPosition()))
                      .linearVelocity(MetersPerSecond.of(inst.getMotorVelocity()));
                },
                inst));
    return routine
        .quasistatic(Direction.kForward)
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.quasistatic(Direction.kReverse))
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kForward))
        .andThen(Commands.waitSeconds(delay))
        .andThen(routine.dynamic(Direction.kReverse))
        .andThen(Commands.waitSeconds(delay));
  }
}
