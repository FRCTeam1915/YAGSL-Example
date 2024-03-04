// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class shooterArticulation extends Command {
  public static TalonSRX motorOne;
  public static TalonSRX motorTwo;
  public static boolean ampPos;
  public static double shooterPos;

  /** Creates a new shooterArticulation. */
  public shooterArticulation(TalonFX motorOne, TalonFX motorTwo, boolean ampPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterPos = Robot.shooterEncoder.getAbsolutePosition();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ampPos) {
      while (shooterPos < 0.707) {
        shooterPos = Robot.shooterEncoder.getAbsolutePosition();
        RobotContainer.armMotorOne.set(.2);
        RobotContainer.armMotorTwo.set(-.2);
        System.out.println("Going to amp!!!!!");
      }
    } else if (!ampPos) {
      while (shooterPos < .502) {
        shooterPos = Robot.shooterEncoder.getAbsolutePosition();
        RobotContainer.armMotorOne.set(-.2);
        RobotContainer.armMotorTwo.set(.2);
        System.out.println("Going to speaker!!!!");
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.armMotorOne.set(0);
    RobotContainer.armMotorTwo.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
