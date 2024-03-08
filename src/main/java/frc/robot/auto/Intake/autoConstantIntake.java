// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.auto.Intake;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooter;
import frc.robot.Robot;

// Run multiple commands in a routine
public class autoConstantIntake extends SequentialCommandGroup {
  double in;
  int time;
  private static TalonSRX motorOne;

  // Routine command constructor
  public autoConstantIntake(TalonSRX motorOne, double in, int time) {
    this.in = in;
    this.time = time;
    this.motorOne = motorOne;
    addCommands(
        new WaitCommand(time),
        new autoConstantIntakeStart(motorOne, in),
        new intakeSensor(),
        new autoConstantIntakeStop(motorOne)

    );
  }

}
