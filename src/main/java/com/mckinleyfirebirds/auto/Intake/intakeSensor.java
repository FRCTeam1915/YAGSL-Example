// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.mckinleyfirebirds.auto.Intake;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import com.mckinleyfirebirds.Robot;

public class intakeSensor extends Command {
    /** Creates a new shooter. */
    boolean finished = false;
    double wait;

    public intakeSensor(double wait) {
        // Use addRequirements() here to declare subsystem dependencies.
        boolean finished = false;
        this.wait = wait;
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
        if (Robot.colorSensor.getIR() > 2000) {
            finished = true;

        }
        if (t_timer.get() > wait) {
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