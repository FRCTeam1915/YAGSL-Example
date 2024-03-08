package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmPIDCmd extends Command {
    private final ArmSubsystem armSubsystem;
    private final PIDController pidController;
    private double setpoint;

    public ArmPIDCmd(ArmSubsystem armSubsystem, double setpoint) {
        this.armSubsystem = armSubsystem;
        this.setpoint = setpoint;
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
        double speed = 0;
        double encval = armSubsystem.getEncoder();
        if (encval > .75 || encval < .4) {
            speed = 0;
        } else {
            speed = pidController.calculate(armSubsystem.getEncoder());
        }
        SmartDashboard.putNumber("Arm speed value -------", speed);
        armSubsystem.setMotor(speed);
        SmartDashboard.putNumber("Arm setpoint value", this.setpoint);
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
