// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.HubScoringConstants.*;
import static frc.robot.Constants.TeleopAssistConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.FeedEject;
import frc.robot.commands.HoldAndIntake;
import frc.robot.commands.IntakeParallel;
import frc.robot.commands.MoveIndexer;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.MoveShooter;
import frc.robot.commands.PivotIntake;
import frc.robot.commands.SetIntakePosition;
import frc.robot.commands.SpeedUpThenIndex;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.telemetry.TelemetryManager;
import frc.robot.util.DriverFeedback;
import frc.robot.util.DriverTuning;
import frc.robot.util.HubShiftEngine;
import frc.robot.util.ShotCalculator;
import java.io.File;
import java.util.List;
import java.util.Set;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final FollowPath.Builder pathBuilder;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController copilotXbox = new CommandXboxController(1);

  final CommandJoystick driverJoystick = new CommandJoystick(2);

  private static RobotContainer instance;

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  Agitator agitator = Agitator.getInstance();
  Indexer indexer = Indexer.getInstance();
  IntakeRoller roller = IntakeRoller.getInstance();
  IntakePivot pivot = IntakePivot.getInstance();
  Shooter shooter = Shooter.getInstance();

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing
  // selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * -1,
              () -> driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** Clones the angular velocity input stream and converts it to a fieldRelative input stream. */
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(() -> -driverXbox.getRightX(), () -> -driverXbox.getRightY())
          .headingWhile(true);

  /** Clones the angular velocity input stream and converts it to a robotRelative input stream. */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX())
          .withControllerRotationAxis(() -> -driverXbox.getRawAxis(2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard =
      driveAngularVelocityKeyboard
          .copy()
          .withControllerHeadingAxis(
              () -> Math.sin(-driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
              () -> Math.cos(-driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
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
    configureBindings();
    registerNamedAutoCommands();
    configureTeleopAssist();
    new EventTrigger("DeployAndIntakeEvent").whileTrue(new HoldAndIntake());
    DriverStation.silenceJoystickConnectionWarning(true);

    ShotCalculator.getInstance().setSwerve(drivebase);

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser("TrenchHumanScore"); // "New New New Auto"

    autoChooser.addOption("Example A (BLine)", pathBuilder.build(new Path("example_a")));
    autoChooser.addOption("Example B (BLine)", pathBuilder.build(new Path("example_b")));

    // Set the default auto (do nothing)
    autoChooser.addOption("Do Nothing", Commands.none());

    // Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

    // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Initialize tunable values (publishes to NetworkTables/Elastic Dashboard)
    DriverTuning.initialize();

    // Wire up telemetry references
    TelemetryManager.getInstance().setVision(drivebase.getVision());
    TelemetryManager.getInstance().setSwerveSubsystem(drivebase);
    TelemetryManager.getInstance().setControllers(driverXbox.getHID(), copilotXbox.getHID());
    DriverFeedback.getInstance().initialize(driverXbox.getHID(), copilotXbox.getHID());

    // Fire control init

    // Pre-spin: auto-spin shooter 5s before hub goes active so RPM is ready at window open.
    // Requires Shooter so it yields to AimAndShootCommand when operator presses RT.
    // runEnd stops the motor when the trigger goes false.
    // new Trigger(
    //         () -> {
    //           var info = HubShiftEngine.getInstance().getOfficialInfo();
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
        "ShakeIntakeWithTimeout",
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

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard =
        drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAngularVelocityKeyboard =
        drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard =
        drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveSetpointGen);
    } else {
      drivebase.setDefaultCommand(driveSetpointGen);
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

      // driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox
          .button(2)
          .whileTrue(
              Commands.runEnd(
                  () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                  () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

      // driverXbox.b().whileTrue(
      //     drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      // );

    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(
          driveFieldOrientedAngularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
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
      driverXbox.y().whileTrue(new MoveShooter(1700));
      // driverXbox.b().whileTrue(new InstantCommand(()->agitator.runVelocity(),(agitator)));
      driverXbox.x().whileTrue(new MoveIndexer(6000));
      driverXbox.rightTrigger().whileTrue(driveFieldOrientedAngularVelocity);
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
      // driverXbox.x().whileTrue(new MoveIndexer(6000));
      driverXbox.b().whileTrue(new MoveIntake(Constants.MotorConstants.DESIRED_INTAKE_RPM));
      driverXbox.a().whileTrue(new IntakeParallel());
      // driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
      // driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      // driverXbox.back().whileTrue(new SetIntakePosition());

      // driverXbox.b().whileTrue(new MoveIntake());
      // driverXbox.y().whileTrue(new MoveIndexer(6000));
      // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock,
      // drivebase).repeatedly());

      // copilotXbox
      //     .y()
      //     .whileTrue(
      //         new AgitateAndIndex(
      //             Constants.MotorConstants.DESIRED_AGITATOR_RPM,
      //             Constants.MotorConstants.DESIRED_INDEXER_RPM);
      // copilotXbox.x().whileTrue(new HoldAndIntake());

      copilotXbox.rightTrigger().whileTrue(new FeedEject());

      copilotXbox.a().whileTrue(new IntakeParallel());
      // driverXbox.b().whileTrue(new MoveShooter(2500));
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
                    var engine = HubShiftEngine.getInstance();
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
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity); // Overrides drive command above!

    driverJoystick.button(12).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    driverJoystick.button(3).onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverJoystick.button(4).whileTrue(drivebase.centerModulesCommand());
    driverJoystick.button(1).onTrue(Commands.none());
    driverJoystick.button(6).onTrue(Commands.none());
  } else {

    // driverJoystick.button(999).onTrue(new DriveToHub(drivebase, getHubCenter(), Constants.HubScoringConstants.SCORING_DISTANCE, getScoringSide(), Constants.HubScoringConstants.SCORING_ARC_WIDTH_DEGREES));
    // driverJoystick.button(12).toggleOnTrue(
    //   new HubArcDrive(drivebase,
    //     driverXbox::getLeftX,
    //     getHubCenter(),
    //     Constants.HubScoringConstants.SCORING_DISTANCE,
    //     getScoringSide()
    //   )
    // );
    //driverJoystick.button(12).onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driverJoystick.button(3).onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverJoystick.button(4).whi(leTrue(drivebase.centerModulesCommand());
    driverJoystick.button(1).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    driverJoystick.button(6).onTrue(Commands.none());
    //driverJoystick.button(999).whileTrue(new RunIntake())
    //    .onFalse(new RetractIntake());
    driverJoystick.button(11).whileTrue(new MoveIndexer(Constants.MotorConstants.DESIRED_INDEXER_RPM));
    driverJoystick.button(999).whileTrue(new MoveShooter(Constants.MotorConstants.DESIRED_SHOOTER_RPM));
    driverJoystick.button(12).whileTrue(new SpeedUpThenIndex());
    driverJoystick.button(12).onFalse(new MoveIndexer(-Constants.MotorConstants.DESIRED_INDEXER_RPM).andThen(Commands.waitSeconds(0.25)).andThen(new MoveIndexer(0)));
  } */

  /**
   * Teleop assist: hold a button, robot pathfinds to the nearest scoring or intake spot. Release
   * the button and you're back to manual control.
   *
   * <p>The robot knows which side of the field it's on via odometry, so it always picks the right
   * spot (HP side vs depot side). Only 2 buttons needed. POV up = go score, POV left = go intake.
   * POV down stays free for emergency dump.
   *
   * <p>Uses pathfindToPoseFlipped so blue-alliance poses auto-flip for red.
   */
  private void configureTeleopAssist() {
    PathConstraints assistConstraints =
        new PathConstraints(
            ASSIST_MAX_VEL_MPS,
            ASSIST_MAX_ACCEL_MPSS,
            ASSIST_MAX_ANGULAR_VEL_RADPS,
            ASSIST_MAX_ANGULAR_ACCEL_RADPSS);

    copilotXbox
        .povUp()
        .whileTrue(
            Commands.either(
                AutoBuilder.pathfindToPoseFlipped(BLUE_HP_SCORING_POSE, assistConstraints)
                    .andThen(
                        Commands.defer(
                            () ->
                                new AimAndShootCommand(
                                    drivebase, shooter, indexer, agitator, () -> 0, () -> 0, false),
                            Set.of(drivebase, shooter, indexer, agitator))),
                AutoBuilder.pathfindToPoseFlipped(BLUE_DEPOT_SCORING_POSE, assistConstraints)
                    .andThen(
                        Commands.defer(
                            () ->
                                new AimAndShootCommand(
                                    drivebase, shooter, indexer, agitator, () -> 0, () -> 0, false),
                            Set.of(drivebase, shooter, indexer, agitator))),
                () ->
                    drivebase
                        .getPose()
                        .nearest(List.of(BLUE_HP_SCORING_POSE, BLUE_DEPOT_SCORING_POSE))
                        .equals(BLUE_HP_SCORING_POSE)));

    copilotXbox
        .povLeft()
        .whileTrue(
            Commands.either(
                AutoBuilder.pathfindToPoseFlipped(BLUE_HP_INTAKE_POSE, assistConstraints)
                    .alongWith(new HoldAndIntake()),
                AutoBuilder.pathfindToPoseFlipped(BLUE_DEPOT_INTAKE_POSE, assistConstraints)
                    .alongWith(new HoldAndIntake()),
                () ->
                    drivebase
                        .getPose()
                        .nearest(List.of(BLUE_HP_INTAKE_POSE, BLUE_DEPOT_INTAKE_POSE))
                        .equals(BLUE_HP_INTAKE_POSE)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autonomous command
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

  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();

    // If no alliance present, assume blue.
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
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
