package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.BallVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ChaseBallCommand extends Command {
    
    private final SwerveSubsystem drive;
    private final BallVision ballVision;

    // Turning gain — how hard to steer toward the ball
    private static final double TURN_KP = 0.04;

    // Forward speed while chasing (0 to 1)
    private static final double CHASE_SPEED = 0.45;

    // Max turn output to prevent spinning out
    private static final double MAX_TURN = 0.6;

    // How many loops (~0.5 sec at 50Hz) we can lose the ball before giving up
    private int lostBallCounter = 0;
    private static final int LOST_BALL_TIMEOUT = 25;

    public ChaseBallCommand(SwerveSubsystem drive, BallVision ballVision) {
        this.drive = drive;
        this.ballVision = ballVision;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        lostBallCounter = 0;
        System.out.println("[ChaseBall] Started — using PhotonVision FUEL ML model.");
        SmartDashboard.putString("Chase Status", "Initializing...");
        SmartDashboard.putString("Chase Mode", "Starting");
    }

    @Override
    public void execute() {
        if (!ballVision.hasBall()) {
            lostBallCounter++;
            // Slowly rotate to search for a ball
            drive.drive(
                new edu.wpi.first.math.geometry.Translation2d(0, 0),
                0.2,
                true
            );
            SmartDashboard.putString("Chase Status", "Searching...");
            SmartDashboard.putString("Chase Mode", "No target");
            return;
        }

        lostBallCounter = 0;

        // Aim for cluster center if multiple balls visible, otherwise single best ball
        double yaw = ballVision.hasCluster()
            ? ballVision.getClusterYaw()
            : ballVision.getBestBallYaw();

        SmartDashboard.putString("Chase Mode", ballVision.hasCluster()
            ? "Cluster (" + ballVision.getBallCount() + " balls)"
            : "Single Ball");

        // P controller — turn proportional to how far off-center the ball is
        double turnOutput = -yaw * TURN_KP;

        // Clamp turn output
        turnOutput = Math.max(-MAX_TURN, Math.min(MAX_TURN, turnOutput));

        // Only drive forward once aimed close enough at target
        double forwardOutput = 0.0;
        if (Math.abs(yaw) < VisionConstants.AIM_TOLERANCE_DEG) {
            forwardOutput = CHASE_SPEED;
            SmartDashboard.putString("Chase Status", "Driving to target");
        } else {
            SmartDashboard.putString("Chase Status", "Aiming...");
        }

        // Drive the swerve — forward in robot-relative X, rotate on Z
        drive.drive(
            new edu.wpi.first.math.geometry.Translation2d(forwardOutput, 0),
            turnOutput,
            false // robot relative
        );

        SmartDashboard.putNumber("Chase Turn Output", turnOutput);
        SmartDashboard.putNumber("Chase Forward Output", forwardOutput);
        SmartDashboard.putNumber("Chase Target Yaw", yaw);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(
            new edu.wpi.first.math.geometry.Translation2d(0, 0),
            0,
            false
        );
        SmartDashboard.putString("Chase Status", interrupted ? "Cancelled" : "Ball Collected!");
        System.out.println("[ChaseBall] Ended. Interrupted: " + interrupted);
    }

    @Override
    public boolean isFinished() {
        // Done when ball fills enough of the frame — we're on top of it
        if (ballVision.hasBall() &&
            ballVision.getBestBallArea() >= VisionConstants.BALL_AREA_THRESHOLD) {
            System.out.println("[ChaseBall] Ball collected!");
            return true;
        }
        // Done if ball has been missing too long
        if (lostBallCounter >= LOST_BALL_TIMEOUT) {
            System.out.println("[ChaseBall] Lost ball — stopping.");
            return true;
        }
        return false;
    }
}
