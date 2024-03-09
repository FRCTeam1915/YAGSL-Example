// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.intake;
import frc.robot.commands.shooter;
import frc.robot.Robot;

public class intakeSensor extends Command {
    /** Creates a new shooter. */
    boolean finished = false;

    public intakeSensor() {
        // Use addRequirements() here to declare subsystem dependencies.
        boolean finished = false;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        boolean finished = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Timer t_timer = new Timer();
        t_timer.restart();
        if (Robot.intakeSensor.get() == true) {
            finished = true;

        }
        if (t_timer.get() > 3) {
            finished = true;
        }

    }

    // Called once the command ends or is interrupted.

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}