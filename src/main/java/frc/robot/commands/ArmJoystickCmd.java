package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmJoystickCmd extends Command {
    private final ArmSubsystem armSubsystem;
    private final double speed;

    public ArmJoystickCmd(ArmSubsystem armSubsystem, double speed) {
        this.armSubsystem = armSubsystem;
        this.speed = speed;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorJoystickCmd started!");
    }

    @Override
    public void execute() {
        armSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setMotor(0);
        System.out.println("ElevatorJoystickCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}