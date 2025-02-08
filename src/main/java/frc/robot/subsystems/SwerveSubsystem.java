// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.Cameras;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SpeedConstants;


import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;

import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static edu.wpi.first.units.Units.Meter;

public class SwerveSubsystem extends SubsystemBase {
  

  public SwerveController swerveController;

  /** Creates a new ExampleSubsystem. */
  // TODO: Delete all references to NetworkTables and DoublePublisher
  // TODO: Create a variable to hold the robot's maximum speed. Default should be 0.8

  /** Creates a new ExampleSubsystem. */
  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("My Pose", Pose2d.struct).publish();
  
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  
  SwerveDrive swerveDrive;
  
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

    swerveController = swerveDrive.swerveController;
    
    RobotConfig config; 
    boolean enableFeedforward = true;
    
    
    try {
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
      
      this::getPose, this::resetOdometry, this::getRobotVelocity, (speedsRobotRelative, moduleFeedForwards) -> {
        if (enableFeedforward) { 
          swerveDrive.drive(speedsRobotRelative, swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative), moduleFeedForwards.linearForces());
        } else {
          swerveDrive.setChassisSpeeds(speedsRobotRelative);
        }
      }, new PPHolonomicDriveController(new PIDConstants(1, 0, 0), new PIDConstants(0.021, 0, 0.2)), config, () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        } 
        return false;
      }, 
      this);  
    } catch (Exception e) {
      System.out.println(e);
    }

    PathfindingCommand.warmupCommand().schedule();
    
  }
  
  public void resetOdometry(Pose2d initialHolonomicPose){
    swerveDrive.resetOdometry(initialHolonomicPose);
  }
  
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();

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
      // TODO: Change scalar value to be a variable to be changed 
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
      swerveDrive.getMaximumChassisVelocity() * translationSpeed));

    });

  }

  public double translationSpeed = 0.8;

  public double changeSpeed(double newSpeed){
    translationSpeed = newSpeed;
    return newSpeed;

      // NetworkTableInstance inst = NetworkTableInstance.getDefault();
      // NetworkTable table = inst.getTable("/SmartDashboard/RobotData");
      
      // xPub = table.getDoubleTopic("x").publish();
      // yPub = table.getDoubleTopic("y").publish();
      
      // xPub.set(headingX.getAsDouble());
      // yPub.set(headingY.getAsDouble());
      
    });
  }
  
  public void getIMU() {
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
    publisher.set(swerveDrive.getPose());
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
  
  
  
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }
  
  public Pose2d getPose() {
    return swerveDrive.getPose();
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
