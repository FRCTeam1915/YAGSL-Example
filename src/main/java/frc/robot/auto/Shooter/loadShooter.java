// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.Shooter;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.Robot;

import frc.robot.auto.Intake.autoConstantIntakeStop;

import frc.robot.auto.Intake.autoIntakeStart;

public class loadShooter extends SequentialCommandGroup {

    public loadShooter(TalonSRX lowerMotor, TalonSRX upperMotor) {

        addCommands(
                new autoIntakeStart(lowerMotor, upperMotor, 0.5),
                new autoShooterStart(0.8),
                new WaitUntilCommand((() -> Robot.intakeSensor.get())),
                new WaitUntilCommand((() -> !Robot.intakeSensor.get())),
                new WaitUntilCommand((() -> Robot.intakeSensor.get())),
                new autoConstantIntakeStop(lowerMotor),
                new autoShooterStop());

    }
}
