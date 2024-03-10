// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.CANSparkFlex;
//import com.revrobotics.CANSparkLowLevel;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.Constants;
import frc.robot.Robot;
//import frc.robot.auto.Shooter.autoShooterStop;
import frc.robot.subsystems.ShooterSubsystem;

public class loadShooter extends Command {
    /** Creates a new shooter. */
    private boolean finished;
    private boolean trig1;
    private boolean trig2;
    private ShooterSubsystem shootersubsystem;

    public loadShooter(ShooterSubsystem shootersubsystem) {
        this.shootersubsystem = shootersubsystem;
        this.trig1 = false;
        this.trig2 = false;
        this.finished = false;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shootersubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        runmotors();
    }

    public void runmotors() {
        shooter.shooterMotorOne.set(0.1);
        shooter.shooterMotorTwo.set(-0.1);
        shooter.shooterMotorOne.setIdleMode(IdleMode.kBrake);
        shooter.shooterMotorTwo.setIdleMode(IdleMode.kBrake);
        intake.motorOne.set(ControlMode.PercentOutput, -.4);
        intake.motorTwo.set(ControlMode.PercentOutput, -.4);
    }

    public void stopmotors() {
        shooter.shooterMotorOne.set(0);
        shooter.shooterMotorTwo.set(0);
        intake.motorOne.set(ControlMode.PercentOutput, 0);
        intake.motorTwo.set(ControlMode.PercentOutput, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /*
         * trig 1 is when we encounter the first edge
         * trig 2 is whe we encounter the hole
         * sensor is the current sensor state
         * 
         * | trig1 | trig2 | sensor |
         * --------------------------
         * | 0 | 0 | 0 | run motors
         * | 0 | 0 | 1 | keep running, set trig 1
         * | 0 | 1 | 0 | NA
         * | 0 | 1 | 1 | NA
         * | 1 | 0 | 0 | keep running, set trig 2, we are in the hole
         * | 1 | 0 | 1 | keep running
         * | 1 | 1 | 0 | keep running
         * | 1 | 1 | 1 | stop motors, we are at the second part
         * 
         */

        boolean sensor = Robot.shooterSensor.get();
        System.out.println("" + trig1 + trig2 + sensor);
        if (trig1 == false && trig2 == false && sensor == false) {
            runmotors();
            finished = false;
        }
        if (trig1 == false && trig2 == false && sensor == true) {
            trig1 = true;
            // System.out.println(" Trig 1" + trig1);
        }
        if (trig1 == true && trig2 == false && sensor == false) {
            trig2 = true;
            // System.out.println("Trig 2");
        }
        if (trig1 == true && trig2 == true && sensor == true) {
            finished = true;
            // System.out.println("Done");
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        stopmotors();
        trig1 = false;
        trig2 = false;
        System.out.println("End load shooter!");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // stopmotors();
        return finished;
    }
}
