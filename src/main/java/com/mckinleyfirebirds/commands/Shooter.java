package com.mckinleyfirebirds.commands;

import com.mckinleyfirebirds.Robot;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends Command {
    final CANSparkFlex motor;
    final int rpm;
    final boolean isClockWise;
    final boolean coast;
    final XboxController xboxController = new XboxController(1);

//    DigitalInput intakeCensor = new DigitalInput(1);

    double speed = 0;

    // This is not the best practice, but since this is a toy project and I do not care
    public Shooter(CANSparkFlex motor, int rpm, boolean isClockWise, boolean coast) {
        this.motor = motor;
        this.rpm = rpm;
        this.isClockWise = isClockWise;
        this.coast = coast;
    }

    @Override
    public void initialize() {
        speed = 1.0;
        if (coast) {
            motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        } else {
            motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        }
    }

    @Override
    public void execute() {
        motor.set(isClockWise ? calculateSpeedWithRPM(rpm, motor.getEncoder().getVelocity()) : -calculateSpeedWithRPM(rpm, motor.getEncoder().getVelocity()));
        if ((this.rpm - Math.abs(this.motor.getEncoder().getVelocity())) < 20) xboxController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
        System.out.println("The current RPM: " + this.motor.getEncoder().getVelocity());
        System.out.println("The abs of rpm = " + Math.abs(this.motor.getEncoder().getVelocity()));
        System.out.println("the target " + this.rpm);

//        if (Robot.shooterCensor.get()) end(true);

//        if (Robot.shooterCensor.get()) {
//            i++;
//        }

//        if ((this.rpm / 2.0) > this.motor.getEncoder().getVelocity()) speed = 0.3;

    }

    @Override
    public void end(boolean interrupted) {
        this.motor.stopMotor();
        speed = 0;
        xboxController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }

    public double calculateSpeedWithRPM(double targetRPM, double lastRPM) {
        if (targetRPM - Math.abs(lastRPM) > 0) this.speed += 0.0001; else this.speed -= 0.0001;
        return this.speed;
    }
}
