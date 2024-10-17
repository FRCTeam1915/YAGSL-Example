package com.mckinleyfirebirds.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.mckinleyfirebirds.Robot;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command {
    TalonSRX motor;
    double speed;

    public Intake(TalonSRX motor, double speed) {
        this.motor = motor;
        this.speed = speed;
    }

    @Override
    public void execute() {
        motor.set(ControlMode.PercentOutput, speed);
        int proximity = Robot.colorSensor.getProximity();
        if (proximity > 2000) end(true);
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(ControlMode.PercentOutput, 0);
    }
}
