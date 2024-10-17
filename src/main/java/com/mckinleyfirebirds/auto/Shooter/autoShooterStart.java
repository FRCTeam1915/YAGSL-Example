// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.mckinleyfirebirds.auto.Shooter;


import com.mckinleyfirebirds.commands.Shooter;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.Command;

public class autoShooterStart extends Command {
  int rpm = 5500;
  CANSparkFlex motor1;
  CANSparkFlex motor2;
  public autoShooterStart(CANSparkFlex motor1, CANSparkFlex motor2) {
      this.motor1 = motor1;
      this.motor2 = motor2;
  }

  public autoShooterStart(double rpm) {
      this.rpm = (int) (rpm * 6000);
  }

  @Override
  public void initialize() {
//      new Shooter(motor1, rpm, true, true);
//      new Shooter(motor2, rpm, false, true);
      motor1.set(0.5);
      motor2.set(0.5);
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
