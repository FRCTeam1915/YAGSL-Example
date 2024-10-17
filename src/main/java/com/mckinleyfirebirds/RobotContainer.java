// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.mckinleyfirebirds;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.mckinleyfirebirds.auto.Intake.autoConstantIntake;
import com.mckinleyfirebirds.auto.Intake.autoIntake;
import com.mckinleyfirebirds.auto.Shooter.*;
import com.mckinleyfirebirds.commands.ArmSubsystem;
import com.mckinleyfirebirds.commands.Intake;
import com.mckinleyfirebirds.commands.RotateArm;
import com.mckinleyfirebirds.commands.ArmPID;
import com.mckinleyfirebirds.commands.Shooter;
import com.mckinleyfirebirds.subsystems.Swerve;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.io.File;

public class RobotContainer {
    Swerve swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));

    public static final TalonSRX intakeLowerMotor = new TalonSRX(Constants.INTAKE_LOWER_MOTOR_ID);
    public static final TalonSRX intakeUpperMotor = new TalonSRX(Constants.INTAKE_UPPER_MOTOR_ID);

    public static final CANSparkFlex motor1 = new CANSparkFlex(33, CANSparkLowLevel.MotorType.kBrushless);
    public static final CANSparkFlex motor2 = new CANSparkFlex(34, CANSparkLowLevel.MotorType.kBrushless);

    ArmSubsystem armSubsystem = new ArmSubsystem();

    TalonFX armMotorOne = new TalonFX(35);
    TalonFX armMotorTwo = new TalonFX(36);

    CommandXboxController driverController = new CommandXboxController(0);
    CommandXboxController shooterController = new CommandXboxController(1);
    SendableChooser<String> autoMode = new SendableChooser<>();


    //    CommandXboxController intakeController = new CommandXboxController(1);

    public RobotContainer() {
        configureBindings();

        // Use lambdas to always get the latest values
        Command driveFieldOrientedAngularVelocity = swerve.driveCommand(() -> -MathUtil.applyDeadband(driverController.getLeftY(), 0.1), () -> -MathUtil.applyDeadband(driverController.getLeftX(), 0.1), () -> -MathUtil.applyDeadband(driverController.getRightX(), 0.3));
        swerve.setDefaultCommand(driveFieldOrientedAngularVelocity);
        SmartDashboard.putData("Autonomous Mode", autoMode);
        autoMode.setDefaultOption("Amp Side", "Blue Amp Side Auto");
        autoMode.addOption("Middle Amp Side", "Blue Middle Amp Auto");
        autoMode.addOption("Middle Not Amp Side", "Blue Middle Not Amp side");
        autoMode.addOption("Not Amp Side", "Blue Not Amp Auto");
        autoMode.addOption("One Note", "One Note Auto");

        NamedCommands.registerCommand("Shooter", new autoShooter(motor1, motor2, 5));
        NamedCommands.registerCommand("Intake", new autoIntake(intakeUpperMotor,intakeLowerMotor,-0.8,1));
        NamedCommands.registerCommand("IntakeConstant", new autoConstantIntake(intakeUpperMotor,-0.8,2));
        NamedCommands.registerCommand("LongConstantIntake", new autoConstantIntake(intakeUpperMotor, -0.8, 3));
        armSubsystem.setDefaultCommand(new RotateArm(armSubsystem, armMotorOne, 0.0));


    }

    // TODO: these keybindings are terrible
    private void configureBindings() {
        motor2.getEncoder().setVelocityConversionFactor(0.5);
        shooterController.rightTrigger().whileTrue(new Shooter(motor1, 5500, true, true));
        shooterController.rightTrigger().whileTrue(new Shooter(motor2, 5500, false, true));

        shooterController.povLeft().whileTrue(new RotateArm(armSubsystem,armMotorOne, -0.025));
        shooterController.povLeft().whileTrue(new RotateArm(armSubsystem,armMotorTwo, -0.025));

        shooterController.povRight().whileTrue(new RotateArm(armSubsystem,armMotorOne, 0.025));
        shooterController.povRight().whileTrue(new RotateArm(armSubsystem,armMotorTwo, 0.025));

        shooterController.povDown().whileTrue(new ArmPID(armSubsystem, Constants.setPoint1).repeatedly());
        shooterController.povUp().whileTrue(new ArmPID(armSubsystem, Constants.setPoint2).repeatedly());


        shooterController.leftTrigger().whileTrue(new Intake(intakeLowerMotor, 0.8));
        shooterController.leftTrigger().whileTrue(new Intake(intakeUpperMotor, -0.8));

        driverController.a().onTrue(Commands.runOnce(swerve.swerveDrive::zeroGyro));

        shooterController.leftBumper().onTrue(new loadShooter(intakeUpperMotor, intakeLowerMotor));
        shooterController.leftBumper().onFalse(new loadShooterOff());
//        driverController.b().whileTrue(Commands.deferredProxy(() -> swerve.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));

        // ===== Intake
        // Turns the lower motor and holds note in intake
        //        this.intakeController.rightBumper().whileTrue(new Intake(intakeLowerMotor,
        // -0.15));
        //
        //        // Turns both upper and lower motor and puts note in shooter
        //        this.intakeController.rightTrigger().whileTrue(new Intake(intakeUpperMotor,
        // -0.15));
        //        this.intakeController.rightTrigger().whileTrue(new Intake(intakeLowerMotor,
        // -0.15));
        //
        //        // Reverses intake motors to drop the note
        //        this.intakeController.leftBumper().whileTrue(new Intake(intakeUpperMotor, 0.15));
        //        this.intakeController.leftBumper().whileTrue(new Intake(intakeLowerMotor, 0.15));
        // =====
    }

    public Command getAutonomousCommand() {
        return swerve.getAutonomousCommand("New Auto");
    }

    public void setDriveMode() {
    }

    public void setMotorBrake(boolean brake) {
        this.swerve.swerveDrive.setMotorIdleMode(brake);
    }
}
