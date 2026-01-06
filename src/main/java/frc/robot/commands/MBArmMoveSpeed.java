package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MBArm;

public class MBArmMoveSpeed extends Command {
    private MBArm arm;
    private double speed;

    public MBArmMoveSpeed(double speed) {
        arm = MBArm.getInstance();
        addRequirements(arm);
        this.speed = speed;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.move(speed);
    }

    @Override
    public void end(boolean interrupted) {
        arm.move(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
