package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HubScoringConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.BallVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.List;

public class CollectAndScoreCommand extends SequentialCommandGroup {

    private final SwerveSubsystem drive;
    private final BallVision ballVision;

    public CollectAndScoreCommand(SwerveSubsystem drive, BallVision ballVision) {
        this.drive = drive;
        this.ballVision = ballVision;

        addCommands(
            // Step 1: Chase and collect the ball
            new ChaseBallCommand(drive, ballVision)
                .withTimeout(10.0), // give up after 10 seconds if ball not collected

            // Step 2: Generate a path on the fly back to the hub and follow it
            Commands.defer(() -> buildReturnToHubCommand(), java.util.Set.of(drive))
        );
    }

    /**
     * Dynamically generates a PathPlanner path from the robot's current pose
     * back to the scoring position in front of the hub.
     */
    private Command buildReturnToHubCommand() {
        Pose2d currentPose = drive.getPose();
        Pose2d hubScoringPose = getHubScoringPose();

        SmartDashboard.putString("Return Target",
            String.format("(%.2f, %.2f)", hubScoringPose.getX(), hubScoringPose.getY()));

        // Path constraints — max velocity, acceleration, angular velocity, angular acceleration
        PathConstraints constraints = new PathConstraints(
            2.5,                          // max velocity m/s
            3.0,                          // max acceleration m/s²
            Units.degreesToRadians(360),  // max angular velocity
            Units.degreesToRadians(540)   // max angular acceleration
        );

        // Build waypoints from current pose to hub scoring pose
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            hubScoringPose
        );

        // If we're already very close to the hub, skip the path
        double distance = currentPose.getTranslation()
            .getDistance(hubScoringPose.getTranslation());

        if (distance < 0.3) {
            System.out.println("[CollectAndScore] Already at hub — skipping path.");
            SmartDashboard.putString("Chase Status", "Already at hub!");
            return Commands.none();
        }

        // Generate path on the fly
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null, // no ideal starting state
            new GoalEndState(0.0, hubScoringPose.getRotation()) // stop at hub, facing correct angle
        );

        // Prevent PathPlanner from flipping — we handle alliance ourselves
        path.preventFlipping = true;

        SmartDashboard.putString("Chase Status", "Returning to hub...");
        System.out.println("[CollectAndScore] Generating path to hub at: " + hubScoringPose);

        return AutoBuilder.followPath(path)
            .andThen(Commands.runOnce(() -> {
                SmartDashboard.putString("Chase Status", "At hub — ready to shoot!");
                System.out.println("[CollectAndScore] Arrived at hub scoring position.");
            }));
    }

    /**
     * Returns the scoring pose in front of the hub based on alliance color.
     * Uses SCORING_DISTANCE from hub center and faces the robot toward the hub.
     */
    private Pose2d getHubScoringPose() {
        boolean isRed = DriverStation.getAlliance()
            .orElse(Alliance.Blue) == Alliance.Red;

        Translation2d hubCenter = isRed
            ? HubScoringConstants.RED_HUB_CENTER
            : HubScoringConstants.BLUE_HUB_CENTER;

        // Position the robot in front of the hub on the alliance's scoring side
        Rotation2d facingHub = isRed
            ? HubScoringConstants.RED_SCORING_SIDE
            : HubScoringConstants.BLUE_SCORING_SIDE;

        // Offset from hub center by scoring distance in the direction the robot faces
        double offsetX = HubScoringConstants.SCORING_DISTANCE * facingHub.getCos();
        double offsetY = HubScoringConstants.SCORING_DISTANCE * facingHub.getSin();

        Translation2d scoringPosition = new Translation2d(
            hubCenter.getX() + offsetX,
            hubCenter.getY() + offsetY
        );

        // Robot faces the hub (opposite of the offset direction)
        Rotation2d robotHeading = facingHub.plus(Rotation2d.fromDegrees(180));

        return new Pose2d(scoringPosition, robotHeading);
    }
}