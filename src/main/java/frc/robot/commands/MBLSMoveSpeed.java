package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MBLinearSlide;

public class MBLSMoveSpeed extends Command {
    private MBLinearSlide ls;
    private double speed;

    public MBLSMoveSpeed(double speed) {
        ls = MBLinearSlide.getInstance();
        addRequirements(ls);
        this.speed = speed;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        ls.move(speed);
    }

    @Override
    public void end(boolean interrupted) {
        ls.move(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
