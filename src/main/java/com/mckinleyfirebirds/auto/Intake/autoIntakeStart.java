// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.mckinleyfirebirds.auto.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;


public class autoIntakeStart extends Command {
    /** Creates a new shooter. */
    double in;
    TalonSRX motorOne;
    TalonSRX motorTwo;

    public autoIntakeStart(TalonSRX motorOne, TalonSRX motorTwo, double in) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.in = in;
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        motorOne.set(ControlMode.PercentOutput, in);
        motorTwo.set(ControlMode.PercentOutput, in);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
