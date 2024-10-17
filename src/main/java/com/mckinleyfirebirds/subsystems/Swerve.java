package com.mckinleyfirebirds.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
    public SwerveDrive swerveDrive;
    double maximumSpeed = Units.feetToMeters(14.5);

    public Swerve(File directory) {
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);

        // Prevent unnecessary objects being created
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        try {
            this.swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (IOException exception) {
            throw new RuntimeException("\"Hey! You forgot to setup the swerve configuration json files in the `deploy` directory!\" - Hao -> " + exception.getMessage());
        }
        // Heading correction should only be used while controlling the robot via angle
        swerveDrive.setHeadingCorrection(false);
        // Disable cosine compensator in simulation
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
        setupPathPlanner();
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(), Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()), Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(), true, false));
    }

    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(swerveDrive.getMaximumVelocity(), 4.0, swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));
        return AutoBuilder.pathfindToPose(pose, constraints, 0.0, 0.0);
    }

    protected void setupPathPlanner() {
        AutoBuilder.configureHolonomic(swerveDrive::getPose, swerveDrive::resetOdometry, swerveDrive::getRobotVelocity, swerveDrive::setChassisSpeeds, new HolonomicPathFollowerConfig(new PIDConstants(0.7, 0, 0), new PIDConstants(0.4, 0, 0.01), 4.5, swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(), new ReplanningConfig()), () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red, this);
    }
    public Command getAutonomousCommand(String pathName){
        return new PathPlannerAuto(pathName);
    }
}
