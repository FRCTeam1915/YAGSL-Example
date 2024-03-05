package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;


public class ArmPIDCmd extends Command{
    private final ArmSubsystem armSubsystem;
    private final PIDController pidController;

    public ArmPIDCmd(ArmSubsystem armSubsystem, double setpoint) {
        this.armSubsystem = armSubsystem;
        this.pidController = new PIDController(//
                ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
        pidController.setSetpoint(setpoint);
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ShooterPIDCmd started!");
        pidController.reset();
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(armSubsystem.getEncoder());
        armSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setMotor(0);
        System.out.println("ElevatorPIDCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }    
}
