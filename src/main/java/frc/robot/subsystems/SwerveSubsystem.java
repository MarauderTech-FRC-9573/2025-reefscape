// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.Cameras;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static edu.wpi.first.units.Units.Meter;

public class SwerveSubsystem extends SubsystemBase {
  
  /** Creates a new ExampleSubsystem. */
  
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  
  SwerveDrive swerveDrive;
  
  // Vision stuff
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  
  //Enable vision odometry updates while driving.
  private final boolean visionDriveTest = true;
  
  //PhotonVision class to keep an accurate odometry.
  private Vision vision;
  
  //To log the pose
  private final Field2d m_field = new Field2d();
  
  
  public SwerveSubsystem() {
    
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.POSE;
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
    
    //Enable Vision if true
    if (visionDriveTest)
    {
      setupPhotonVision();
      // Stop the odometry thread if we are using vision that way we can synchronize updates better.
      swerveDrive.stopOdometryThread();
    }
    
    SmartDashboard.putData("Field", m_field);
    
  }
  
  // Setup the photon vision class.
  public void setupPhotonVision()
  {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
    System.out.println("Photon Vision Setup");
    
  }
  
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
  DoubleSupplier headingY)
  {
    //swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      
      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
      translationY.getAsDouble()), 0.8);
      
      //Constantly update the values
      SmartDashboard.putNumber("headingX", headingX.getAsDouble());
      SmartDashboard.putNumber("headingY", headingY.getAsDouble());
      SmartDashboard.putNumber("setpoint", swerveDrive.swerveController.lastAngleScalar);
      
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
      headingX.getAsDouble(),
      headingY.getAsDouble(),
      swerveDrive.getOdometryHeading().getRadians(),
      swerveDrive.getMaximumChassisVelocity()));
      
    });
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // When vision is enabled we must manually update odometry in SwerveDrive
    if (visionDriveTest)
    {
      swerveDrive.updateOdometry();
      vision.updatePoseEstimation(swerveDrive);
    }
    
    m_field.setRobotPose(swerveDrive.getPose());
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  public Command aimAtTarget(Cameras camera)
  {
    
    return run(() -> {
      try { 
        Optional<PhotonPipelineResult> resultO = camera.getBestResult();
        if (resultO.isPresent())
        {
          var result = resultO.get();
          if (result.hasTargets())
          {
            swerveDrive.drive(getTargetSpeeds(0,
            0,
            Rotation2d.fromDegrees(result.getBestTarget()
            .getYaw()))); // Not sure if this will work, more math may be required.
          }  
        } 
      } catch (Exception e) {
        System.out.println("No targets found");
      }
    });
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
  
  /**
  * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
  * 90deg.
  *
  * @param xInput X joystick input for the robot to move in the X direction.
  * @param yInput Y joystick input for the robot to move in the Y direction.
  * @param angle  The angle in as a {@link Rotation2d}.
  * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
  */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
    scaledInputs.getY(),
    angle.getRadians(),
    getHeading().getRadians(),
    swerveDrive.getMaximumChassisVelocity());
  }
  
  /**
  * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
  * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
  *
  * @return The yaw angle
  */
  public Rotation2d getHeading()
  {
    return swerveDrive.getPose().getRotation();
  }
}
