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
        Timer t2_timer = new Timer();
        t2_timer.restart();
        shooter.shooterMotorOne.set(0.1);
        shooter.shooterMotorTwo.set(-0.1);
        shooter.shooterMotorOne.setIdleMode(IdleMode.kBrake);
        shooter.shooterMotorTwo.setIdleMode(IdleMode.kBrake);
        intake.motorOne.set(ControlMode.PercentOutput, -.3);
        intake.motorTwo.set(ControlMode.PercentOutput, -.3);
        while (Robot.shooterSensor.get() != true && t2_timer.get() < 6) {
        }
        while (Robot.shooterSensor.get()) {
        }
        while (Robot.shooterSensor.get() != true && t2_timer.get() < 8) {
        }
        shooter.shooterMotorOne.set(0);
        shooter.shooterMotorTwo.set(0);
        intake.motorOne.set(ControlMode.PercentOutput, 0);
        intake.motorTwo.set(ControlMode.PercentOutput, 0);
        t2_timer.stop();

    }

    // Called once the command ends or is interrupted.

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
