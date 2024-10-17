package com.mckinleyfirebirds.auto.Intake;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class autoConstantIntake extends SequentialCommandGroup {
    double in;
    int time;
    TalonSRX motorOne;

    public autoConstantIntake(TalonSRX motorOne, double in, int time) {
    this.in = in;
    this.time = time;
    this.motorOne = motorOne;
    addCommands(new WaitCommand(time), new autoConstantIntakeStart(motorOne, in), new intakeSensor(5), new autoConstantIntakeStop(motorOne));
    }

}
