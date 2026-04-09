// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.HubScoringConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AgitateAndIndex;
import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.FeedEject;
import frc.robot.commands.HoldAndIntake;
import frc.robot.commands.HubArcDrive;
import frc.robot.commands.MoveAgitator;
import frc.robot.commands.PivotIntake;
import frc.robot.commands.SetIntakePosition;
import frc.robot.commands.SpeedUpThenIndex;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.sim.SimDriveOverride;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.telemetry.TelemetryManager;
import frc.robot.util.DriverFeedback;
import frc.robot.util.DriverTuning;
import java.io.File;
import java.util.Set;
import swervelib.SwerveInputStream;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
   private final AutoFactory autoFactory;
  private final FollowPath.Builder pathBuilder;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController copilotXbox = new CommandXboxController(1);

  final CommandJoystick driverJoystick = new CommandJoystick(2);
  private SendableChooser<Command> autoChooser;
  Shooter shooter = Shooter.getInstance();
  Indexer indexer = Indexer.getInstance();
  Agitator agitator = Agitator.getInstance();
  IntakePivot intakePivot = IntakePivot.getInstance();
  IntakeRoller intakeRoller = IntakeRoller.getInstance();
  Hanger hanger = Hanger.getInstance();
  private boolean useLeftOffset = true;
  private static RobotContainer instance;

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  private final Field2d field = new Field2d();

  // NOTE: This creates a second Vision instance. SwerveSubsystem already creates one internally.
  // Both call getAllUnreadResults() on the same Cameras singletons, causing a race condition.
  // Safe to remove this line and use drivebase.getVision() everywhere instead.
  // public final Vision visionSubsystem = new Vision(drivebase::getPose, field);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity. In sim, SimDriveOverride values are combined with joystick so scenario playback and
   * manual SimGUI control both work.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () ->
                  RobotBase.isSimulation()
                      ? -(driverXbox.getLeftY() + SimDriveOverride.getY())
                      : driverXbox.getLeftY() * -1,
              () ->
                  RobotBase.isSimulation()
                      ? -(driverXbox.getLeftX() + SimDriveOverride.getX())
                      : driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(
              () ->
                  RobotBase.isSimulation()
                      ? -(driverXbox.getRightX() + SimDriveOverride.getOmega())
                      : driverXbox.getRightX() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** Clone's the angular velocity input stream and converts it to a fieldRelative input stream. */
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
          .headingWhile(true);

  /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX())
          .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard =
      driveAngularVelocityKeyboard
          .copy()
          .withControllerHeadingAxis(
              () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
              () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
          .headingWhile(true)
          .translationHeadingOffset(true)
          .translationHeadingOffset(Rotation2d.fromDegrees(0));

  //   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
  //   () -> driverJoystick.getY() * -1,
  //   () -> driverJoystick.getX() * -1)
  //   .withControllerRotationAxis(driverJoystick::getTwist)
  //   .deadband(OperatorConstants.DEADBAND)
  //   .scaleTranslation(0.8)
  //   .allianceRelativeControl(true);

  // SwerveInputStream driveAngularVelocityKeyboard =
  // SwerveInputStream.of(drivebase.getSwerveDrive(),
  //   () -> -driverJoystick.getY(),
  //   () -> -driverJoystick.getX())
  //   .withControllerRotationAxis(() -> driverJoystick.getTwist())
  //   .deadband(OperatorConstants.DEADBAND)
  //   .scaleTranslation(0.8)
  //   .allianceRelativeControl(true);
  // // Derive the heading axis with math!
  // SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
  //   .withControllerHeadingAxis(() -> Math.sin(driverJoystick.getTwist() * Math.PI) * (Math.PI *
  // 2),
  //       () -> Math.cos(driverJoystick.getTwist() * Math.PI) * (Math.PI * 2))
  //
  // .headingWhile(true).translationHeadingOffset(true).translationHeadingOffset(Rotation2d.fromDegrees(0));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private RobotContainer() {

    autoFactory = new AutoFactory(
            drivebase::getPose, // A function that returns the current robot pose
            drivebase::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            drivebase::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            drivebase // The drive subsystem
        );
    pathBuilder =
        new FollowPath.Builder(
                drivebase,
                drivebase::getPose,
                drivebase::getRobotVelocity,
                drivebase::setChassisSpeeds, // Consumer to drive the robot
                new PIDController(5.0, 0.0, 0.0), // Translation PID
                new PIDController(3.0, 0.0, 0.0), // Rotation PID
                new PIDController(2.0, 0.0, 0.0) // Cross-track PID
                )
            .withDefaultShouldFlip() // Auto-flip for red alliance
            .withPoseReset(drivebase::resetOdometry); // Reset odometry at path start
    // Configure the trigger bindings
    registerNamedAutoCommands();

    autoChooser = AutoBuilder.buildAutoChooser("TrenchHumanScore"); // "New New New Auto"
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // drivebase.setupPathPlanner();
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    configureBindings();
    new EventTrigger("DeployAndIntakeEvent").whileTrue(new HoldAndIntake());
    DriverStation.silenceJoystickConnectionWarning(true);

    frc.robot.util.ShotCalculator.getInstance().setSwerve(drivebase);

    // Build an auto chooser. This will use Commands.none() as the default option.
    // Initialize tunable values (publishes to NetworkTables/Elastic Dashboard)
    DriverTuning.initialize();

    // Wire up telemetry references
    TelemetryManager.getInstance().setVision(drivebase.getVision());
    TelemetryManager.getInstance().setSwerveSubsystem(drivebase);
    TelemetryManager.getInstance().setControllers(driverXbox.getHID(), copilotXbox.getHID());
    DriverFeedback.getInstance().initialize(driverXbox.getHID(), copilotXbox.getHID());
    // autoChooser = AutoBuilder.buildAutoChooser("TrenchHumanScore"); // "New New New Auto"

    // Fire control init

    // Pre-spin: auto-spin shooter 5s before hub goes active so RPM is ready at window open.
    // Requires Shooter so it yields to AimAndShootCommand when operator presses RT.
    // runEnd stops the motor when the trigger goes false.
    // new Trigger(
    //         () -> {
    //           var info = frc.robot.util.HubShiftEngine.getInstance().getOfficialInfo();
    //           return !info.hubActive()
    //               && info.timeToNextActive() > 0
    //               && info.timeToNextActive() < 5.0;
    //         })
    //     .whileTrue(
    //         Commands.runEnd(
    //             () ->
    //                 Shooter.getInstance()
    //                     .moveToVelocityWithPID(Shooter.getInstance().getTunableTargetRPM()),
    //             () -> Shooter.getInstance().move(0),
    //             Shooter.getInstance()));

    // Hub deactivation warning is handled inside DriverFeedback.update() so it
    // doesn't compete with other haptic patterns for HID rumble output.
  }

  private void registerNamedAutoCommands() {
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("DeployIntake", new DeployIntake());

    NamedCommands.registerCommand("HoldAndRunIntake", new HoldAndIntake());
    NamedCommands.registerCommand("HoldAndRunIntakeTimed", new HoldAndIntake().withTimeout(4));

    NamedCommands.registerCommand("SpeedUpThenShoot", new SpeedUpThenIndex());
    NamedCommands.registerCommand("TimedShoot", new SpeedUpThenIndex().withTimeout(8));

    NamedCommands.registerCommand(
        "ShakeIntake",
        (new PivotIntake(-0.3).withTimeout(.89).andThen(new PivotIntake(0.2).withTimeout(.7)))
            .repeatedly());
    NamedCommands.registerCommand(
        "ShakeIntake",
        (new PivotIntake(-0.3).withTimeout(.89).andThen(new PivotIntake(0.2).withTimeout(.7)))
            .repeatedly()
            .withTimeout(8));
    NamedCommands.registerCommand(
        "ShakeIntakeAndScore",
        ((new PivotIntake(-0.3).withTimeout(.89).andThen(new PivotIntake(0.2).withTimeout(.7)))
                .repeatedly())
            .alongWith(
                new AimAndShootCommand(
                    drivebase,
                    shooter,
                    indexer,
                    agitator,
                    () -> -driverXbox.getLeftY() * -1,
                    () -> -driverXbox.getLeftX() * -1,
                    false)));
    NamedCommands.registerCommand(
        "ShakeIntakeAndScoreWithTimeout",
        ((new PivotIntake(-0.3).withTimeout(.89).andThen(new PivotIntake(0.2).withTimeout(.7)))
                .repeatedly())
            .alongWith(
                new AimAndShootCommand(
                    drivebase,
                    shooter,
                    indexer,
                    agitator,
                    () -> -driverXbox.getLeftY() * -1,
                    () -> -driverXbox.getLeftX() * -1,
                    true))
            .withTimeout(3.67));

    NamedCommands.registerCommand("shoot", new SpeedUpThenIndex());

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser("TrenchHumanScore"); // "New New New Auto"

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    autoChooser.addOption("Example A (BLine)", pathBuilder.build(new Path("example_a")));
    autoChooser.addOption("Example B (BLine)", pathBuilder.build(new Path("example_b")));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Initialize tunable values (publishes to NetworkTables/Elastic Dashboard)
    DriverTuning.initialize();

    // Wire up telemetry references
    TelemetryManager.getInstance().setVision(drivebase.getVision());
    TelemetryManager.getInstance().setSwerveSubsystem(drivebase);
    TelemetryManager.getInstance().setControllers(driverXbox.getHID(), copilotXbox.getHID());
    DriverFeedback.getInstance().initialize(driverXbox.getHID(), copilotXbox.getHID());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    Command hubArcDrive =
        Commands.defer(
            () ->
                new HubArcDrive(
                    drivebase,
                    driverXbox::getLeftX,
                    getHubCenter(),
                    SCORING_DISTANCE,
                    getScoringSide()),
            Set.of(drivebase));

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard =
        drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard =
        drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard =
        drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else {
      drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(
          () -> target,
          new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
          new ProfiledPIDController(
              5, 0, 0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
      driverXbox
          .start()
          .onTrue(
              Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      // Button B
      driverXbox.button(3).onTrue(Commands.runOnce(() -> toggleOffset()));

      // driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox
          .button(2)
          .whileTrue(
              Commands.runEnd(
                  () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                  () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

      // driverXbox.b().whileTrue(
      // drivebase.driveToPose(
      // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      // );

    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(
          driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {

      // driverXbox
      //     .a()
      //     .onTrue(
      //         Commands.defer(
      //             () ->
      //                 HubScoringUtil.driveToHubCommand(
      //                     drivebase,
      //                     getHubCenter(),
      //                     SCORING_DISTANCE,
      //                     getScoringSide(),
      //                     SCORING_ARC_WIDTH_DEGREES),
      //             Set.of(drivebase)));

      driverXbox.rightBumper().whileTrue(new PivotIntake(-0.2));
      driverXbox.leftBumper().whileTrue(new PivotIntake(0.2));
      // driverXbox.y().whileTrue(new MoveShooter(1500));
      // driverXbox.b().whileTrue(new InstantCommand(()->agitator.runVelocity(),(agitator)));
      // driverXbox.x().whileTrue(new MoveIndexer(5000));
      driverXbox.rightTrigger().whileTrue(driveFieldOrientedAnglularVelocity);
      driverXbox
          .leftTrigger()
          .whileTrue(
              new AimAndShootCommand(
                  drivebase,
                  shooter,
                  indexer,
                  agitator,
                  () -> -driverXbox.getLeftY() * -1,
                  () -> -driverXbox.getLeftX() * -1,
                  false));
      // driverXbox.y().whileTrue(new RetractIntake());
      driverXbox.x().whileTrue(new MoveAgitator());
      driverXbox.a().whileTrue(new HoldAndIntake());
      driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
      // driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.back().whileTrue(new SetIntakePosition());
      // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock,
      // drivebase).repeatedly());
      // driverXbox.povDown().whileTrue(SysId.agitatorSysIdCommand());
      // driverXbox.povLeft().whileTrue(SysId.indexerSysIdCommand());
      // driverXbox.povUp().whileTrue(SysId.shooterSysIdCommand());
      // driverXbox.povRight().whileTrue(SysId.intakePivotSysIdCommand());
      // driverXbox.rightBumper().whileTrue(SysId.intakeRollerSysIdCommand());
      // driverXbox.leftBumper().whileTrue(SysId.hangerSysIdCommand());
      // driverXbox.rightTrigger().whileTrue(drivebase.sysIdAngleMotorCommand());
      // driverXbox.leftTrigger().whileTrue(drivebase.sysIdDriveMotorCommand());

      // copilotXbox
      //     .y()
      //     .whileTrue(
      //         new AgitateAndIndex(
      //             Constants.AgitatorConstants.TARGET_RPM,
      //             Constants.IndexerConstants.TARGET_SPEED,
      //             hubArcDrive::isScheduled));
      // copilotXbox.x().whileTrue(new HoldAndIntake());

      copilotXbox.rightTrigger().whileTrue(new FeedEject());
      copilotXbox.a().whileTrue(new DeployIntake().andThen(new HoldAndIntake()));
      copilotXbox.b().whileTrue(new AgitateAndIndex(-5000, -5000));
      copilotXbox.leftBumper().whileTrue(new PivotIntake(0.3));
      copilotXbox.rightBumper().whileTrue(new PivotIntake(-0.3));
      // copilotXbox
      //     .rightTrigger()
      //     .whileTrue(
      //         new AimAndShootCommand(
      //             drivebase,
      //             Shooter.getInstance(),
      //             Indexer.getInstance(),
      //             Agitator.getInstance(),
      //             () -> -driverXbox.getLeftY(),
      //             () -> -driverXbox.getLeftX(),
      //             false));
      copilotXbox
          .leftTrigger()
          .whileTrue(
              (new PivotIntake(-0.3).withTimeout(.89).andThen(new PivotIntake(0.2).withTimeout(.7)))
                  .repeatedly());

      // second controller Start toggles wonAuto override (fixes bad FMS data mid-match)
      // second controller Back clears override and returns to FMS-derived schedule
      copilotXbox
          .start()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    var engine = frc.robot.util.HubShiftEngine.getInstance();
                    engine.setWonAutoOverride(!engine.isWonAuto());
                  }));
      copilotXbox.back().onTrue(new SetIntakePosition());

      // emergency dump: flat RPM, immediate feed, bypasses everything
      // copilotXbox
      //     .povDown()
      //     .whileTrue(
      //         Commands.runEnd(
      //             () -> {
      //               Shooter.getInstance()
      //                   .moveToVelocityWithPID(Constants.ShooterConstants.EMERGENCY_DUMP_RPM);
      //               Indexer.getInstance()
      //                   .moveToVelocityWithPID(Indexer.getInstance().getTunableTargetSpeed());
      //               Agitator.getInstance().runVelocity();
      //             },
      //             () -> {
      //               Shooter.getInstance().move(0);
      //               Indexer.getInstance().move(0);
      //               Agitator.getInstance().runVelocity();
      //             },
      //             Shooter.getInstance(),
      //             Indexer.getInstance(),
      //             Agitator.getInstance()));

      // //       Trigger crossingZone = new Trigger(()->{
      //     Pose2d pose = drivebase.getPose();
      //
      // if(pose.getTranslation().getX()<RED_HUB_CENTER.getX()+1&&pose.getTranslation().getX()>RED_HUB_CENTER.getX()-1||
      //
      // pose.getTranslation().getX()<BLUE_HUB_CENTER.getX()+1&&pose.getTranslation().getX()>BLUE_HUB_CENTER.getX()-1){
      //     return true;
      //     }
      //     else{
      //     return false;
      //     }
      // });
      // crossingZone.whileTrue(Commands.run(() -> {
      //   new DeployIntake();
      //     }
      // ));

      //     }
    }
  }

  // Rotation2d current = drivebase.getHeading();
  // Rotation2d target;

  // if (Math.abs(current.getDegrees()) < 90 || Math.abs(current.getDegrees()) > 270) {
  //     target = Rotation2d.fromDegrees(0);
  // } else {
  //     target = Rotation2d.fromDegrees(180);
  // }
  //     ChassisSpeeds speeds = drivebase.getTargetSpeeds(
  //         driverXbox.getLeftY(),
  //         driverXbox.getLeftX(),
  //         target.getSin(),
  //         target.getCos()
  //     );
  //     drivebase.driveFieldOriented(speeds);
  // }, drivebase));

  /*     if (DriverStation.isTest()) {
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

    driverJoystick.button(12).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    driverJoystick.button(11).whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
    driverJoystick.button(3).onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverJoystick.button(4).whileTrue(drivebase.centerModulesCommand());
    driverJoystick.button(1).onTrue(Commands.none());
    driverJoystick.button(6).onTrue(Commands.none());
  } else {
    driverJoystick.button(12).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    driverJoystick.button(11).whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
    driverJoystick.button(3).onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverJoystick.button(4).whileTrue(drivebase.centerModulesCommand());
    driverJoystick.button(1).onTrue(Commands.none());
    driverJoystick.button(6).onTrue(Commands.none());
  } */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return autoChooser.getSelected()
    return autoChooser.getSelected();
  }

  public Command driveToTag(int tagId) {
    Transform2d offset =
        new Transform2d(
            new Translation2d(0.75, 0.0), // 0.75 meters in front of tag
            Rotation2d.fromDegrees(180) // face tag
            );

    Pose2d targPose = Vision.getAprilTagPose(tagId, offset);
    if (targPose == null) {
      return new InstantCommand(); // do nothing
    }

    return drivebase.driveToPose(targPose);
  }

  public boolean getUseLeftOffset() {
    return useLeftOffset;
  }

  public void toggleOffset() {
    useLeftOffset = !useLeftOffset;
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return drivebase;
  }

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }

    return instance;
  }

  public Cameras getBestCamera(int id) {
    // Replace this with the actual logic to get the best camera
    return drivebase.getVision().getbestCamera(id);
  }

  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  private Translation2d getHubCenter() {
    boolean isRedAlliance = isRedAlliance();

    if (isRedAlliance) {
      return RED_HUB_CENTER;
    } else {
      return BLUE_HUB_CENTER;
    }
  }

  private Rotation2d getScoringSide() {
    boolean isRedAlliance = isRedAlliance();

    if (isRedAlliance) {
      return RED_SCORING_SIDE;
    } else {
      return BLUE_SCORING_SIDE;
    }
  }
}
