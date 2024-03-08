// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.intake;
import frc.robot.commands.shooter;
import frc.robot.auto.Shooter.autoShooterStop;

public class loadShooter extends Command {
    /** Creates a new shooter. */
    boolean trig1 = false;
    boolean trig2 = false;
    boolean cycle = false;
    public static boolean finished = false;

    public loadShooter() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    public void runmotors() {
        shooter.shooterMotorOne.set(0.1);
        shooter.shooterMotorTwo.set(-0.1);
        shooter.shooterMotorOne.setIdleMode(IdleMode.kBrake);
        shooter.shooterMotorTwo.setIdleMode(IdleMode.kBrake);
        intake.motorOne.set(ControlMode.PercentOutput, -.4);
        intake.motorTwo.set(ControlMode.PercentOutput, -.4);
    }
    public void stopmotors () {
        shooter.shooterMotorOne.set(0);
        shooter.shooterMotorTwo.set(0);
        intake.motorOne.set(ControlMode.PercentOutput, 0);
        intake.motorTwo.set(ControlMode.PercentOutput, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //shooter.shooterMotorOne.set(0.1);
        //shooter.shooterMotorTwo.set(-0.1);
        //shooter.shooterMotorOne.setIdleMode(IdleMode.kBrake);
        //shooter.shooterMotorTwo.setIdleMode(IdleMode.kBrake);
        //intake.motorOne.set(ControlMode.PercentOutput, -.4);
        //intake.motorTwo.set(ControlMode.PercentOutput, -.4);

        /*
         * trig 1 is when we encounter the first edge
         * trig 2 is whe we encounter the hole
         * sensor is the current sensor state
         * 
         *   | trig1 | trig2 | sensor |
         *   --------------------------
         *   |   0   |   0   |    0   |  run motors
         *   |   0   |   0   |    1   |  keep running, set trig 1
         *   |   0   |   1   |    0   |  NA
         *   |   0   |   1   |    1   |  NA
         *   |   1   |   0   |    0   |  keep running, set trig 2, we are in the hole
         *   |   1   |   0   |    1   |  keep running 
         *   |   1   |   1   |    0   |  keep running
         *   |   1   |   1   |    1   |  stop motors, we are at the second part
         * 
         */

        boolean sensor = Robot.shooterSensor.get();
        if (trig1 == false) {
            if (trig2 == false) {
                runmotors();
                if (sensor == true) {
                    trig1 = true;
                }
            }
        } else if (trig1 == true) {
            if (trig2 == false && sensor == false) {
                runmotors();
                trig2 = true;
            } else if (trig2 = true && sensor == true) {
                stopmotors();
                finished = true;
            } else {
                runmotors();
            }
        }
/*
        if (Robot.shooterSensor.get() == false) {
            cycle = true;
        }
        if (Robot.shooterSensor.get() == true && finished == false) {
            if (trig1 == false && trig2 == false) {
                trig1 = true;
            } else if (trig1 == true && trig2 == false && cycle == true) {
                trig2 = true;
            } else if (trig1 == true && trig2 == true) {
                trig1 = false;
                trig2 = false;
                cycle = false;
                finished = true;
                shooter.shooterMotorOne.set(0);
                shooter.shooterMotorTwo.set(0);
                intake.motorOne.set(ControlMode.PercentOutput, 0);
                intake.motorTwo.set(ControlMode.PercentOutput, 0);
            }
        }
*/
    }

    // Called once the command ends or is interrupted.

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        stopmotors();
        trig1 = false;
        trig2 = false;
        return finished;
    }
}
