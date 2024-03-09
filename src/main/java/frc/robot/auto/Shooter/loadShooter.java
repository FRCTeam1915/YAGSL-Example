// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.Shooter;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.auto.Intake.autoConstantIntakeStop;
import frc.robot.auto.Intake.autoIntakeStart;
import frc.robot.auto.Intake.autoIntakeStop;

public class loadShooter extends SequentialCommandGroup {

    public loadShooter(TalonSRX lowerMotor, TalonSRX upperMotor) {

        addCommands(
                // starts shooter and intake
                new autoShooterStart(0.1),
                new autoIntakeStart(lowerMotor, upperMotor, 0.5),
                // waitsfor sensor to find notes
                new WaitUntilCommand((() -> Robot.intakeSensor.get())),
                // waits for sensor to not find notes
                new WaitUntilCommand((() -> !Robot.intakeSensor.get())),
                // waits for sensor to find notes
                new WaitUntilCommand((() -> Robot.intakeSensor.get())),
                // stops both motors
                new autoShooterStop(),
                new autoIntakeStop(lowerMotor, upperMotor));

    }
}
