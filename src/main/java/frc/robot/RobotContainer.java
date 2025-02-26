// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberSetPos;
import frc.robot.commands.Intake_SetPos;
import frc.robot.commands.Score_SetPos;
import frc.robot.commands.Startup_SetPos;
import frc.robot.commands.ClimberManualPower;
import frc.robot.commands.elevator.ElevatorManualPower;
import frc.robot.commands.elevator.ElevatorSetPos;
import frc.robot.commands.manipulator.AlgaeIntake_SetPower;
import frc.robot.commands.manipulator.CoralIntake_SetPower;
import frc.robot.commands.manipulator.Wrist_SetPos;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulators;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.DoubleSupplier;

import swervelib.SwerveInputStream;
import swervelib.parser.SwerveDriveConfiguration;

import static swervelib.math.SwerveMath.calculateMaxAngularVelocity;

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
  final CommandXboxController gunnerXbox = new CommandXboxController(1);// change as needed
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));
  // private final Manipulators manipulators = new Manipulators();
  private final ElevatorManualPower elevatorUp = new ElevatorManualPower(3);
  private final ElevatorManualPower elevatorDown = new ElevatorManualPower(-3);
  private final ClimberManualPower climberUp = new ClimberManualPower(4.0);
  private final ClimberManualPower climberDown = new ClimberManualPower(-4.0);
  public static double currentSpeed = Constants.MAX_SPEED;
  public boolean lowSpeed = false;
  // private final SwerveSubsystem drivebase = new SwerveSubsystem(new
  // File("/Users/3042/Documents/GitHub/Reefscape-2025/src/main/deploy",
  // "swerve"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  // .withControllerRotationAxis(driverXbox::getRightX)

  private SendableChooser<Command> autoChooser;
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY(), () -> driverXbox.getLeftX())// change back to -1
      .withControllerRotationAxis(getDriverXboxRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  private DoubleSupplier getDriverXboxRightX() {
    return () -> driverXbox.getRightX() * -1;
  }

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
      .withControllerHeadingAxis(
          () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
          () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("test print", Commands.print("test"));
  }

  public void toggleSlowMode() {
    if (!lowSpeed) {
      currentSpeed = Constants.LOW_MAX_SPEED;
      lowSpeed = true;
      System.out.println(currentSpeed);

    } else {
      currentSpeed = Constants.MAX_SPEED;
      lowSpeed = false;
      System.out.println(currentSpeed);
    }
    SwerveDriveConfiguration driveCfg = drivebase.getSwerveDriveConfiguration();
    drivebase.getSwerveDrive().setMaximumAllowableSpeeds(currentSpeed,
        calculateMaxAngularVelocity(
            currentSpeed,
            Math.abs(driveCfg.moduleLocationsMeters[0].getX()),
            Math.abs(driveCfg.moduleLocationsMeters[0].getY())));
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

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      // gunnerXbox.leftBumper().whileTrue((elevatorDown));
      // gunnerXbox.rightBumper().whileTrue((elevatorUp));

    } else { // left bumper toggles slowmode for driver(IMPORTANT)
      /*
       * EVERYTHING BELOW IS FOR GUNNER
       * left/right bumpers are coral intake/expel for gunner
       * left/right triggers are algae intake/expel
       * a: elevator down & wrist down
       * x: elevator lowest level & wrist downn
       * b: elevator mid level & wrist down
       * y: elevator highest level & wrist down
       * left/right joystick is wrist up/down - ask manipulator (or dpad???) ASK ONCE
       * THEY DECIDE RAHH
       * left/right joystick is climber up/down -ask manipulator (or dpad???) ASK ONCE
       * THEY DECIDE RAHH
       */
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // driverXbox.b().whileTrue(
      // drivebase.driveToPose(
      // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(new InstantCommand(() -> toggleSlowMode()));
      driverXbox.leftTrigger().whileTrue((climberDown));
      driverXbox.rightTrigger().whileTrue((climberUp));
      driverXbox.povDown().onTrue(new ClimberSetPos(200));
      driverXbox.povUp().onTrue(new ClimberSetPos(0));
      driverXbox.b().whileTrue((elevatorDown));
      driverXbox.y().whileTrue((elevatorUp));

      // gunner code
      // changed to setVoltageCoralPower, may need to change back depending on limit
      // switch :)
      gunnerXbox.leftBumper().whileTrue(new CoralIntake_SetPower(-4));
      gunnerXbox.rightBumper().whileTrue(new CoralIntake_SetPower(3));
      gunnerXbox.leftTrigger().whileTrue(new AlgaeIntake_SetPower(3));
      gunnerXbox.rightTrigger().whileTrue(new AlgaeIntake_SetPower(-3));
      gunnerXbox.a().onTrue(new Score_SetPos(ElevatorConstants.L1EncoderCounts));
      gunnerXbox.x().onTrue(new Score_SetPos(ElevatorConstants.L2EncoderCounts));
      gunnerXbox.b().onTrue(new Score_SetPos(ElevatorConstants.L3EncoderCounts));
      gunnerXbox.y().onTrue(new Score_SetPos(ElevatorConstants.L4EncoderCounts));
      gunnerXbox.leftStick().onTrue(new Intake_SetPos());
      gunnerXbox.rightStick().onTrue(new Startup_SetPos());
      gunnerXbox.povUp().onTrue((new Intake_SetPos()));
      gunnerXbox.povDown().onTrue((new Startup_SetPos()));

      // gunnerXbox.b().onTrue(new Startup_SetPos());
      // gunnerXbox.a().onTrue(new Intake_SetPos());

    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("Ne Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
// hi!
// bye!