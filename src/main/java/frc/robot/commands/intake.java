// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class intake extends Command {
  /** Creates a new pickUp. */
  public static TalonSRX motorOne;
  public static TalonSRX motorTwo;
  double speed;

  public intake(TalonSRX motorOne, TalonSRX motorTwo, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motorOne.set(ControlMode.PercentOutput, speed);
    motorTwo.set(ControlMode.PercentOutput, -speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motorOne.set(ControlMode.PercentOutput, 0);
    motorTwo.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
