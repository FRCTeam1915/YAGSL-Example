// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.mckinleyfirebirds.auto.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.mckinleyfirebirds.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class loadShooterOff extends Command {
    public loadShooterOff() {

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.motor1.stopMotor();
        RobotContainer.motor2.stopMotor();
        RobotContainer.intakeLowerMotor.set(ControlMode.PercentOutput, 0);
        RobotContainer.intakeUpperMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
