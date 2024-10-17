package com.mckinleyfirebirds.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmSubsystem extends SubsystemBase {
    private TalonFX motorOne = new TalonFX(35);
    private TalonFX motorTwo = new TalonFX(36);
    public DutyCycleEncoder encoder = new DutyCycleEncoder(2);

    public ArmSubsystem() {
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm encoder value", encoder.getAbsolutePosition());
    }

    public void setMotor(double speed) {
        motorOne.set(speed);
        motorTwo.set(-speed);
    }

    public double getEncoder() {
        return encoder.getAbsolutePosition();
    }
}