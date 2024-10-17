// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.mckinleyfirebirds.auto.Shooter;


import com.mckinleyfirebirds.commands.Shooter;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.Command;

import com.mckinleyfirebirds.RobotContainer;

public class autoShooterStop extends Command {
  CANSparkFlex motor1;
  CANSparkFlex motor2;

  public autoShooterStop(CANSparkFlex motor1, CANSparkFlex motor2) {
      this.motor1 = motor1;
      this.motor2 = motor2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      motor1.stopMotor();
      motor2.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
