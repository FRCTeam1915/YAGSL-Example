// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  public boolean sensorTrig1;
  public boolean sensorTrig2;

  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    //CANSparkFlex shooterMotorOne = new CANSparkFlex(Constants.shooterMotorOne,
    //  CANSparkLowLevel.MotorType.kBrushless);
    //CANSparkFlex shooterMotorTwo = new CANSparkFlex(Constants.shooterMotorTwo,
    //  CANSparkLowLevel.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
