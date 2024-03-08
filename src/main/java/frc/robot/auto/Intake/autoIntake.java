// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.auto.Intake;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Run multiple commands in a routine
public class autoIntake extends SequentialCommandGroup {
  double in;
  int time;
  private static TalonSRX motorOne;
  private static TalonSRX motorTwo;

  // Routine command constructor
  public autoIntake(TalonSRX motorOne, TalonSRX motorTwo, double in, int time) {
    this.in = in;
    this.time = time;
    this.motorOne = motorOne;
    this.motorTwo = motorTwo;
    addCommands(
        new WaitCommand(1.5),
        new autoIntakeStart(motorOne, motorTwo, in),
        new intakeSensor(),
        new autoIntakeStop(motorOne, motorTwo)

    );
  }

}
