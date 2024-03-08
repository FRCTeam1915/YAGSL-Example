// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.intake;
import frc.robot.commands.shooter;
import frc.robot.auto.Shooter.autoShooterStop;

public class loadShooter extends Command {
    /** Creates a new shooter. */
    public static boolean trig1 = false;
    public static boolean trig2 = false;
    public static boolean cycle = false;
    public static boolean finished = false;

    public loadShooter() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Robot.shooterSensor.get() == false && trig1 == false && trig2 == false) {
            shooter.shooterMotorOne.set(0.1);
            shooter.shooterMotorTwo.set(-0.1);
            shooter.shooterMotorOne.setIdleMode(IdleMode.kBrake);
            shooter.shooterMotorTwo.setIdleMode(IdleMode.kBrake);
            intake.motorOne.set(ControlMode.PercentOutput, -.4);
            intake.motorTwo.set(ControlMode.PercentOutput, -.4);
        }
        if (Robot.shooterSensor.get() == false && trig1 && trig2 == false) {
            trig2 = true;
        }
        if (Robot.shooterSensor.get() && finished == false) {
            if (trig1 == false && trig2 == false) {
                trig1 = true;
                shooter.shooterMotorOne.set(0.1);
                shooter.shooterMotorTwo.set(-0.1);
                shooter.shooterMotorOne.setIdleMode(IdleMode.kBrake);
                shooter.shooterMotorTwo.setIdleMode(IdleMode.kBrake);
                intake.motorOne.set(ControlMode.PercentOutput, -.4);
                intake.motorTwo.set(ControlMode.PercentOutput, -.4);
            } else if (trig1 && trig2) {
                cycle = true;
            } else if (trig1 && trig2 && cycle) {

                finished = true;
                shooter.shooterMotorOne.set(0);
                shooter.shooterMotorTwo.set(0);
                intake.motorOne.set(ControlMode.PercentOutput, 0);
                intake.motorTwo.set(ControlMode.PercentOutput, 0);
            }
        }
    }

    // Called once the command ends or is interrupted.

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}
