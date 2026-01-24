// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToTag;
import frc.robot.commands.DriveToHub;
import frc.robot.commands.HubArcDrive;
//import frc.robot.commands.AlignWithAprilTag;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveInputStream;
import frc.robot.Constants;
import frc.robot.Constants.HubScoringConstants;
import java.io.File;
import java.util.function.BooleanSupplier;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveInputStream;
import static frc.robot.Constants.HubScoringConstants.*;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.File;
import java.util.function.BooleanSupplier;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandJoystick driverJoystick = new CommandJoystick(1);
  private final SendableChooser<Command> autoChooser;
  
  private boolean useLeftOffset = true;

  private static RobotContainer instance;

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));
   // Initialize the vision subsystem

  private final Field2d field = new Field2d();

  public final Vision visionSubsystem = new Vision(drivebase::getPose, field);
  /**
   * 
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
          () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
      .headingWhile(true).translationHeadingOffset(true).translationHeadingOffset(Rotation2d.fromDegrees(0));

//   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
//   () -> driverJoystick.getY() * -1,
//   () -> driverJoystick.getX() * -1)
//   .withControllerRotationAxis(driverJoystick::getTwist)
//   .deadband(OperatorConstants.DEADBAND)
//   .scaleTranslation(0.8)
//   .allianceRelativeControl(true);




// SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
//   () -> -driverJoystick.getY(),
//   () -> -driverJoystick.getX())
//   .withControllerRotationAxis(() -> driverJoystick.getTwist())
//   .deadband(OperatorConstants.DEADBAND)
//   .scaleTranslation(0.8)
//   .allianceRelativeControl(true);
// // Derive the heading axis with math!
// SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
//   .withControllerHeadingAxis(() -> Math.sin(driverJoystick.getTwist() * Math.PI) * (Math.PI * 2),
//       () -> Math.cos(driverJoystick.getTwist() * Math.PI) * (Math.PI * 2))
//   .headingWhile(true).translationHeadingOffset(true).translationHeadingOffset(Rotation2d.fromDegrees(0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    
    driverXbox.y().onTrue(Commands.runOnce(() -> toggleOffset()));

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
          Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
          new ProfiledPIDController(
              5,
              0,
              0,
              new Constraints(5, 2)),
          new ProfiledPIDController(
              5,
              0,
              0,
              new Constraints(
                  Units.degreesToRadians(360),
                  Units.degreesToRadians(180))));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
       // Button B
      driverXbox.button(3).onTrue(Commands.runOnce(() ->toggleOffset()));     
           
      //driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(
          Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
              () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

      // driverXbox.b().whileTrue(
      // drivebase.driveToPose(
      // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      // );
      
    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      
      driverXbox.a().onTrue(new DriveToHub(drivebase, getHubCenter(), SCORING_DISTANCE, getScoringSide(), SCORING_ARC_WIDTH_DEGREES));
      driverXbox.x().toggleOnTrue(
        new HubArcDrive(drivebase,
          driverXbox::getLeftX,
          getHubCenter(),
          SCORING_DISTANCE,
          getScoringSide()
        )
      );
      //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      //driverXbox.rightBumper().onTrue(new AlignWithAprilTag());
    }
  }
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
    return autoChooser.getSelected();      
  }

  public Command driveToTag(int tagId) {
    Transform2d offset = new Transform2d(
        new Translation2d(0.75, 0.0),  // 0.75 meters in front of tag
        Rotation2d.fromDegrees(180)    // face tag
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
    return visionSubsystem.getbestCamera(id);
  }

  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
  
    // Default to blue if no alliance data
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


