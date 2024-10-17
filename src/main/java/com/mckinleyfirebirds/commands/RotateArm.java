package com.mckinleyfirebirds.commands;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateArm extends Command {
    private final ArmSubsystem armSubsystem;
    TalonFX motor;
    double speed;


    public RotateArm(ArmSubsystem armSubsystem,TalonFX motor, double speed) {
        this.armSubsystem = armSubsystem;
        this.motor = motor;
        this.speed = speed;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        motor.set(this.speed);
    }

    @Override
    public void end(boolean interrupted) {
        motor.stopMotor();
    }
}
