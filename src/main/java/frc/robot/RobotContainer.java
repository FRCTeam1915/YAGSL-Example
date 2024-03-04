// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.auto.Intake.autoIntake;
import frc.robot.auto.Intake.autoPickUp;
import frc.robot.auto.Shooter.autoShooter;
import frc.robot.auto.Shooter.loadShooter;
import frc.robot.auto.Intake.autoConstantIntake;
import frc.robot.commands.climbing;
import frc.robot.commands.intake;
import frc.robot.commands.motorArm;
import frc.robot.commands.shooter;
import frc.robot.commands.shooterArticulation;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController intakeXbox = new CommandXboxController(1);

  // Motors defined
  public static TalonSRX lowerMotor = new TalonSRX(Constants.pickUpID);
  public static TalonSRX upperMotor = new TalonSRX(Constants.upperMotorID);
  public static TalonFX armMotorOne = new TalonFX(Constants.armMotorOne);
  public static TalonFX armMotorTwo = new TalonFX(Constants.armMotorTwo);
  public static TalonSRX climbMotorOne = new TalonSRX(Constants.climbOneID);
  public static TalonSRX climbMotorTwo = new TalonSRX(Constants.climbTwoID);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public static SendableChooser<String> autoMode = new SendableChooser<>();

  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();

    SmartDashboard.putData("Autonomous Setting", autoMode);
    autoMode.addOption("Amp Side", "Amp Side Auto");
    // autoMode.setDefaultOption("Position 2 - Middle", "");
    autoMode.addOption("Not Amp Side", "Not Amp Auto");

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
            OperatorConstants.RIGHT_X_DEADBAND),
        driverXbox.getHID()::getYButtonPressed,
        driverXbox.getHID()::getAButtonPressed,
        driverXbox.getHID()::getXButtonPressed,
        driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> -driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(// swag DRIP swag
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRawAxis(2), OperatorConstants.RIGHT_X_DEADBAND));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

    NamedCommands.registerCommand("Shooter", new autoShooter(.8, 2));
    NamedCommands.registerCommand("Intake", new autoIntake(upperMotor, lowerMotor, -.8, 1));
    NamedCommands.registerCommand("IntakeConstant", new autoConstantIntake(lowerMotor, -0.8, 2));
    NamedCommands.registerCommand("LongIntakeConstant", new autoConstantIntake(lowerMotor, -0.8, 4));
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
            new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock,
    // drivebase).repeatedly());

    // Intake and shooter commands
    // Turns the lower motor and holds note in intake
    Trigger rightBumper = intakeXbox.rightBumper();
    rightBumper.onTrue(new autoPickUp(lowerMotor, -0.7));

    // Turns both upper and lower motor and puts note in shooter
    Trigger rightTrigger = intakeXbox.rightTrigger();
    rightTrigger.whileTrue(new intake(lowerMotor, upperMotor, -.75));

    // Reverses intake motors to drop the note
    Trigger leftBumper = intakeXbox.leftBumper();
    leftBumper.whileTrue(new intake(lowerMotor, upperMotor, .5));

    // Articulates shooter so it can shoot into the amp
    // Moves shooter down
    Trigger povDown = intakeXbox.povDown();
    povDown.whileTrue(new shooterArticulation(armMotorOne, armMotorTwo, true));
    // Moves shooter up
    Trigger povUp = intakeXbox.povUp();
    povUp.whileTrue(new shooterArticulation(armMotorOne, armMotorTwo, false));
    // Moves shooter down slowly
    Trigger povLeft = intakeXbox.povLeft();
    povLeft.whileTrue(new motorArm(armMotorOne, armMotorTwo, .025));
    // Moves shooter up slowly
    Trigger povRight = intakeXbox.povRight();
    povRight.whileTrue(new motorArm(armMotorOne, armMotorTwo, -.025));

    // Shooter commands
    // Shoots note for speaker
    Trigger AButton = intakeXbox.a();
    AButton.whileTrue(new shooter(.8));
    // Shoots note for amp
    Trigger Ybutton = intakeXbox.y();
    Ybutton.whileTrue(new shooter(.1));
    // Shoots note reverse
    Trigger Bbutton = intakeXbox.b();
    Bbutton.whileTrue(new shooter(-.1));
    Trigger Xbutton = intakeXbox.x();
    Xbutton.onTrue(new loadShooter());

    // Climbing commands
    // Climb up
    Trigger climbUp = driverXbox.leftTrigger();
    climbUp.whileTrue(new climbing(climbMotorOne, climbMotorTwo, 1));
    // Climb down
    Trigger climbDown = driverXbox.rightTrigger();
    climbDown.whileTrue(new climbing(climbMotorOne, climbMotorTwo, -1));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Amp Side Auto");
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
