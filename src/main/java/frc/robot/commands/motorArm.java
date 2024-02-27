// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;

public class motorArm extends Command {
  private static com.ctre.phoenix6.hardware.TalonFX  motorOne;
  private static com.ctre.phoenix6.hardware.TalonFX  motorTwo;
  double speed = 0;

  /** Creates a new motorArm. */
  public motorArm(com.ctre.phoenix6.hardware.TalonFX motorOne, com.ctre.phoenix6.hardware.TalonFX motorTwo, double speed) {
    this.motorOne = motorOne;
    this.motorTwo = motorOne;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motorOne.set(speed);
    motorTwo.set(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motorOne.set(0);
    motorTwo.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
