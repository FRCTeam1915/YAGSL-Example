// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.mckinleyfirebirds.auto.Shooter;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.mckinleyfirebirds.RobotContainer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import com.mckinleyfirebirds.Robot;
import com.mckinleyfirebirds.auto.Intake.*;


public class loadShooter extends SequentialCommandGroup {
    public loadShooter(TalonSRX lowerMotor, TalonSRX upperMotor) {
        addCommands(
                // starts shooter and intake
                new autoShooterStart(0.1),
                new autoIntakeStart(lowerMotor, upperMotor, 0.5),
                // waitsfor sensor to find notes
                new WaitUntilCommand((() -> Robot.colorSensor.getProximity() > 2000)),
                // waits for sensor to not find notes
                new WaitUntilCommand((() -> !(Robot.colorSensor.getProximity() > 2000))),
                // waits for sensor to find notes
                new WaitUntilCommand((() -> Robot.colorSensor.getProximity() > 2000)),
                // stops both motors
                new autoShooterStop(RobotContainer.motor1, RobotContainer.motor2),
                new autoIntakeStop(lowerMotor, upperMotor));

    }
}
