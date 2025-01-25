package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


import static edu.wpi.first.units.Units.Meter;

public class SwerveSubsystem extends SubsystemBase {
    
    File directory = new File(Filesystem.getDeployDirectory(),"swerve");
    
    SwerveDrive swerveDrive;
    
    public SwerveSubsystem(){
        
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try
        {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed,
            new Pose2d(new Translation2d(Meter.of(1),
            Meter.of(4)),
            Rotation2d.fromDegrees(0)));
            // Alternative method if you don't want to supply the conversion factor via JSON files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e)
        {
            throw new RuntimeException(e);
        }
        
    }
    
    /**
    * Example command factory method.
    *
    * @return a command
    */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
        () -> {
            /* one-time action goes here */
        });
    }
    
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
    DoubleSupplier headingY)
    {
        //swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
        return run(() -> {
            
            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
            translationY.getAsDouble()), 0.8);
            
            SmartDashboard.putNumber("headingX", headingX.getAsDouble());
            SmartDashboard.putNumber("headingY", headingY.getAsDouble());
            //Voodoo magic to make the setpoint match the value we read from IMU(It doesn't work)
            SmartDashboard.putNumber("setpoint", swerveDrive.swerveController.lastAngleScalar);
            
            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
            headingX.getAsDouble(),
            headingY.getAsDouble(),
            swerveDrive.getOdometryHeading().getRadians(),
            swerveDrive.getMaximumChassisVelocity()));
            
            
        });
    }
    
    
    /**
    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
    *
    * @return value of some boolean subsystem state, such as a digital sensor.
    */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
    
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }
    
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }
}
