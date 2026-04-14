package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HubScoringConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShotCalculatorConstants;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.telemetry.SafeLog;
import frc.robot.util.HeadingController;

public class ShootFromCenterCommand extends Command {

    private final SwerveSubsystem swerve;
    private final Shooter shooter;
    private final Indexer indexer;
    private final Agitator agitator;

    private final HeadingController headingController = new HeadingController();
    private final Timer shootTimer = new Timer();
    private final Timer spinupTimeoutTimer = new Timer();
    private final Timer reverseTimer = new Timer();

    // max/min distance we allow shooting from
    private static final double MAX_SHOT_DISTANCE_M = 4.0;
    private static final double MIN_SHOT_DISTANCE_M = ShotCalculatorConstants.MIN_SCORING_DISTANCE;

    private static final double SHOOT_DURATION_SEC = 3.0;
    private static final double SPINUP_TIMEOUT_SEC = 3.5;
    private static final double REVERSE_DURATION_SEC = 0.65;
    private static final double HEADING_TOLERANCE_DEG = 3.0;
    private static final double OMEGA_LIMIT_FRACTION = 0.4;
    private static final double OLOCK_OMEGA_THRESHOLD = 0.15;

    // states for the command
    private enum State {
        VALIDATING,
        TURNING,
        REVERSING,
        SPINNING_UP,
        SHOOTING,
        DONE
    }

    private State currentState = State.VALIDATING;
    private Rotation2d targetHeading = new Rotation2d();
    private final ChassisSpeeds reusableSpeeds = new ChassisSpeeds();
    private boolean feeding = false;

    public ShootFromCenterCommand(
            SwerveSubsystem swerve,
            Shooter shooter,
            Indexer indexer,
            Agitator agitator) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.indexer = indexer;
        this.agitator = agitator;
        addRequirements(swerve, shooter, indexer, agitator);
    }

    @Override
    public void initialize() {
        currentState = State.VALIDATING;
        feeding = false;
        headingController.reset();

        // figure out which way to face at the start
        targetHeading = computeAllianceSideFacing();

        SmartDashboard.putString("ShootCenter/Status", "Validating...");
    }

    @Override
    public void execute() {
        switch (currentState) {
            case VALIDATING:  handleValidating();  break;
            case TURNING:     handleTurning();     break;
            case REVERSING:   handleReversing();   break;
            case SPINNING_UP: handleSpinningUp();  break;
            case SHOOTING:    handleShooting();    break;
            case DONE:                             break;
        }

        SmartDashboard.putString("ShootCenter/State", currentState.toString());
        SmartDashboard.putNumber("ShootCenter/ShooterRPM", shooter.getVelocityRPM());
        SmartDashboard.putNumber("ShootCenter/DistanceM", getShotDistance());
        SmartDashboard.putNumber("ShootCenter/HeadingErrorDeg",
            Math.toDegrees(headingController.getPositionError()));
    }

    private void handleValidating() {
        double distance = getShotDistance();

        // make sure we know our alliance
        if (DriverStation.getAlliance().isEmpty()) {
            fail("Alliance unknown");
            return;
        }
        // don't shoot if too far or too close
        if (distance > MAX_SHOT_DISTANCE_M) {
            fail("Too far: " + distance + "m");
            return;
        }
        if (distance < MIN_SHOT_DISTANCE_M) {
            fail("Too close: " + distance + "m");
            return;
        }

        // start spinning up while we turn to save time
        shooter.moveToVelocityWithPID(ShooterConstants.TARGET_RPM);
        SmartDashboard.putString("ShootCenter/Status", "Turning...");
        currentState = State.TURNING;
    }

    private void handleTurning() {
        double maxOmega = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();

        double omega = headingController.calculate(
            swerve.getHeading(),
            targetHeading,
            0.0,
            getShotDistance(),
            maxOmega * OMEGA_LIMIT_FRACTION
        );

        shooter.moveToVelocityWithPID(ShooterConstants.TARGET_RPM);

        // stay still, just rotate
        reusableSpeeds.vxMetersPerSecond = 0;
        reusableSpeeds.vyMetersPerSecond = 0;
        reusableSpeeds.omegaRadiansPerSecond = omega;
        swerve.drive(reusableSpeeds, Translation2d.kZero);

        double errorDeg = Math.abs(targetHeading.minus(swerve.getHeading()).getDegrees());
        SmartDashboard.putNumber("ShootCenter/TurnErrorDeg", errorDeg);

        // close enough, move on
        if (errorDeg < HEADING_TOLERANCE_DEG) {
            agitator.moveToVelocityWithPID(-5000);
            indexer.moveToVelocityWithPID(-5000);
            reverseTimer.restart();
            SmartDashboard.putString("ShootCenter/Status", "Clearing jams...");
            currentState = State.REVERSING;
        }
    }

    private void handleReversing() {
        // hold heading while we reverse to clear jams
        double maxOmega = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();
        double omega = headingController.calculate(
            swerve.getHeading(),
            targetHeading,
            0.0,
            getShotDistance(),
            maxOmega * OMEGA_LIMIT_FRACTION
        );

        reusableSpeeds.vxMetersPerSecond = 0;
        reusableSpeeds.vyMetersPerSecond = 0;
        reusableSpeeds.omegaRadiansPerSecond = omega;
        swerve.drive(reusableSpeeds, Translation2d.kZero);

        shooter.moveToVelocityWithPID(ShooterConstants.TARGET_RPM);

        if (reverseTimer.hasElapsed(REVERSE_DURATION_SEC)) {
            spinupTimeoutTimer.restart();
            SmartDashboard.putString("ShootCenter/Status", "Spinning up...");
            currentState = State.SPINNING_UP;
        }
    }

    private void handleSpinningUp() {
        double maxOmega = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();
        double omega = headingController.calculate(
            swerve.getHeading(),
            targetHeading,
            0.0,
            getShotDistance(),
            maxOmega * OMEGA_LIMIT_FRACTION
        );

        // x-lock if we dont need to rotate
        if (Math.abs(omega) < OLOCK_OMEGA_THRESHOLD) {
            swerve.lock();
        } else {
            reusableSpeeds.vxMetersPerSecond = 0;
            reusableSpeeds.vyMetersPerSecond = 0;
            reusableSpeeds.omegaRadiansPerSecond = omega;
            swerve.drive(reusableSpeeds, Translation2d.kZero);
        }

        shooter.moveToVelocityWithPID(ShooterConstants.TARGET_RPM);

        if (shooter.isAtSpeed(ShooterConstants.TARGET_RPM)) {
            feeding = true;
            shootTimer.restart();
            SmartDashboard.putString("ShootCenter/Status", "Firing!");
            currentState = State.SHOOTING;
            return;
        }

        if (spinupTimeoutTimer.hasElapsed(SPINUP_TIMEOUT_SEC)) {
            fail("Shooter didnt spin up in time");
        }
    }

    private void handleShooting() {
        double maxOmega = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();
        double omega = headingController.calculate(
            swerve.getHeading(),
            targetHeading,
            0.0,
            getShotDistance(),
            maxOmega * OMEGA_LIMIT_FRACTION
        );

        if (Math.abs(omega) < OLOCK_OMEGA_THRESHOLD) {
            swerve.lock();
        } else {
            reusableSpeeds.vxMetersPerSecond = 0;
            reusableSpeeds.vyMetersPerSecond = 0;
            reusableSpeeds.omegaRadiansPerSecond = omega;
            swerve.drive(reusableSpeeds, Translation2d.kZero);
        }

        shooter.moveToVelocityWithPID(ShooterConstants.TARGET_RPM);

        // stop feeding if rpm drops too much
        double threshold = ShooterConstants.TARGET_RPM * 0.85;
        if (shooter.getVelocityRPM() < threshold) {
            feeding = false;
            indexer.move(0);
            agitator.moveToVelocityWithPID(agitator.getTunableTargetRPM() * 0.1);
        } else {
            feeding = true;
            indexer.moveToVelocityWithPID(indexer.getTunableTargetSpeed());
            agitator.moveToVelocityWithPID(5990);
        }

        SmartDashboard.putNumber("ShootCenter/TimeLeft", SHOOT_DURATION_SEC - shootTimer.get());

        if (shootTimer.hasElapsed(SHOOT_DURATION_SEC)) {
            SmartDashboard.putString("ShootCenter/Status", "Done!");
            currentState = State.DONE;
        }
    }

    // turn so the rear of the robot faces our alliance side
    // we use the same shooter angle offset as AimAndShootCommand
    private Rotation2d computeAllianceSideFacing() {
        boolean isRed = DriverStation.getAlliance()
            .orElse(Alliance.Blue) == Alliance.Red;

        Rotation2d facingOurSide = isRed
            ? Rotation2d.fromDegrees(180)
            : Rotation2d.fromDegrees(0);

        return facingOurSide.plus(new Rotation2d(ShotCalculatorConstants.SHOOTER_ANGLE_OFFSET_RAD));
    }

    private double getShotDistance() {
        Pose2d robotPose = swerve.getPose();
        return robotPose.getTranslation().getDistance(getHubCenter());
    }

    private Translation2d getHubCenter() {
        boolean isRed = DriverStation.getAlliance()
            .orElse(Alliance.Blue) == Alliance.Red;
        return isRed
            ? HubScoringConstants.RED_HUB_CENTER
            : HubScoringConstants.BLUE_HUB_CENTER;
    }

    private void fail(String reason) {
        currentState = State.DONE;
        System.err.println("[ShootFromCenter] Failed: " + reason);
        SmartDashboard.putString("ShootCenter/Status", "Failed: " + reason);
        SafeLog.put("ShootFromCenter/FailReason", reason);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.move(0);
        indexer.move(0);
        agitator.move(0);
        SmartDashboard.putString("ShootCenter/Status", interrupted ? "Interrupted" : "Complete");
        SafeLog.put("ShootFromCenter/Active", false);
    }

    @Override
    public boolean isFinished() {
        return currentState == State.DONE;
    }
}