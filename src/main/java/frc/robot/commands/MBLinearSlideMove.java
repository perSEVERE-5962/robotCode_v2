package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MBLinearSlide;

/* Move Mini Build linear slide to certain position */
public class MBLinearSlideMove extends Command {
    private MBLinearSlide ls;
    private double pos;

    /* -1 <= pos <= 1 (usually) */
    public MBLinearSlideMove(double pos) {
        ls = MBLinearSlide.getInstance();
        addRequirements(ls);
        this.pos = pos;
    }

    @Override
    public void execute() {
        ls.moveToPositionWithPID(pos);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
