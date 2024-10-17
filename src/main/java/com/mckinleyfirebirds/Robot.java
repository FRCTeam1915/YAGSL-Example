// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.mckinleyfirebirds;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import com.revrobotics.ColorSensorV3;

public class Robot extends TimedRobot {
    RobotContainer robotContainer;
    Command autonomousCommand;
    DigitalInput shooterCensor = new DigitalInput(0);
    DigitalInput intakeCensor = new DigitalInput(1);
    public static I2C.Port i2cPort = I2C.Port.kOnboard;
    public static ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

//        Color detectedColor = colorSensor.getColor();
        int proximity = colorSensor.getProximity();
//        System.out.println(proximity);
        Color detectedColor = colorSensor.getColor();

        Color color = new Color("#955612");
//        System.out.println(detectedColor.toHexString().equals(color.toHexString()) ? "true" : "");
//        System.out.println(detectedColor.toHexString());

//        SmartDashboard.putData("Color Sensor", detectedColor);
    }

    @Override
    public void disabledInit() {
        robotContainer.setMotorBrake(true);
    }

    @Override
    public void autonomousInit() {
        robotContainer.setMotorBrake(true);
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null){
            autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        robotContainer.setDriveMode();
        robotContainer.setMotorBrake(true);
        if (autonomousCommand != null){
            autonomousCommand.cancel();
        }
    }
}