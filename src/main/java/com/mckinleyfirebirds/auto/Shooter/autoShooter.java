// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package com.mckinleyfirebirds.auto.Shooter;


import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Run multiple commands in a routine
public class autoShooter extends SequentialCommandGroup {

  int time;

  // Routine command constructor
  public autoShooter(CANSparkFlex motor1, CANSparkFlex motor2, int time) {

    this.time = time;

    addCommands(new autoShooterStart(motor1, motor2), new WaitCommand(time), new autoShooterStop(motor1, motor2));
  }

}
