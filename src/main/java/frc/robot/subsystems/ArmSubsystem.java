package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;


public class ArmSubsystem extends SubsystemBase {
    private TalonFX motorOne = new TalonFX(ArmConstants.armMotorOne);
    private TalonFX motorTwo = new TalonFX(ArmConstants.armMotorTwo);
    public DutyCycleEncoder encoder = new DutyCycleEncoder(//
            ArmConstants.kEncoderChannel);

    public ArmSubsystem() {
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm encoder value", encoder.get());
    }

    public void setMotor(double speed) {
        motorOne.set(speed);
        motorTwo.set(speed);
    }

    public double getEncoder() {
        return encoder.getAbsolutePosition();
    }
}