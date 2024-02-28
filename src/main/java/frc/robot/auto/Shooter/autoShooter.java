// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.auto.Shooter;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.Shooter.autoShooterStart;
import frc.robot.auto.Shooter.autoShooterStop;
import frc.robot.commands.shooter;

// Run multiple commands in a routine
public class autoShooter extends SequentialCommandGroup {
  double in;
  int time;

  // Routine command constructor
  public autoShooter(double in, int time) {
    this.in = in;
    this.time = time;

    addCommands(
        new autoShooterStart(in),
        new WaitCommand(time),
        new autoShooterStop()

    );
  }

}
