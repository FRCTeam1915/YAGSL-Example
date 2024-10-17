// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package com.mckinleyfirebirds.auto.Intake;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Run multiple commands in a routine
public class autoPickUp extends SequentialCommandGroup {
  double in;

  private static TalonSRX motorOne;

  // Routine command constructor
  public autoPickUp(TalonSRX motorOne, double in) {
    this.in = in;

    this.motorOne = motorOne;
    addCommands(

        new autoConstantIntakeStart(motorOne, in),
        new intakeSensor(5),
        new autoConstantIntakeStop(motorOne)

    );
  }

}
