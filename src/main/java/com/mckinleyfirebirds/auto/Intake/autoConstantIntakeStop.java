// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.mckinleyfirebirds.auto.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;


public class autoConstantIntakeStop extends Command {
    /** Creates a new shooter. */
    private static TalonSRX motorOne;

    public autoConstantIntakeStop(TalonSRX motorOne) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.motorOne = motorOne;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        motorOne.set(ControlMode.PercentOutput, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {



    }

    // Called once the command ends or is interrupted.

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
