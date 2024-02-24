// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class shooter extends Command {
  /** Creates a new shooter. */
  public static CANSparkFlex shooterMotorOne = new CANSparkFlex(Constants.shooterMotorOne, CANSparkLowLevel.MotorType.kBrushless);
  public static CANSparkFlex shooterMotorTwo = new CANSparkFlex(Constants.shooterMotorTwo, CANSparkLowLevel.MotorType.kBrushless);
  double in;

  public shooter(double in) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.in = in;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterMotorOne.set(in);
    shooterMotorTwo.set(-in);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterMotorOne.set(0);
    shooterMotorTwo.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
