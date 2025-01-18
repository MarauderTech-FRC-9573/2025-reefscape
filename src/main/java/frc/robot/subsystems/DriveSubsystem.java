package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;;


/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
* RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
*
* The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
* for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
* that hardware is only being used by 1 command at a time.
*/
public class DriveSubsystem extends SubsystemBase {
    /*Class member variables. These variables represent things the class needs to keep track of and use between
    different method calls. */
    DifferentialDrive m_drivetrain;
    
    
    private final SparkMax leftFront = new SparkMax(kLeftFrontID, SparkLowLevel.MotorType.kBrushed);
    private final SparkMax leftRear = new SparkMax(kLeftRearID, SparkLowLevel.MotorType.kBrushed);
    private final SparkMax rightFront = new SparkMax(kRightFrontID, SparkLowLevel.MotorType.kBrushed);
    private final SparkMax rightRear = new SparkMax(kRightRearID, SparkLowLevel.MotorType.kBrushed);

    private final SparkMaxConfig leftFrontConfig = new SparkMaxConfig();
    private final SparkMaxConfig leftRearConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightFrontConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightRearConfig = new SparkMaxConfig();
    
    // private final DifferentialDriveOdometry m_odometry;
    
    // Gains are for example purposes only - must be determined for your own robot!
    
    // Gyroscope
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    
    Pose2d m_pose;
    
    /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
    * member variables and perform any configuration or set up necessary on hardware.
    */
    public DriveSubsystem() {
        
        leftFrontConfig.smartCurrentLimit(kDriveCurrentLimit);
        leftRearConfig.smartCurrentLimit(kDriveCurrentLimit);
        rightFrontConfig.smartCurrentLimit(kDriveCurrentLimit);
        rightRearConfig.smartCurrentLimit(kDriveCurrentLimit);
        
        // Set the rear motors to follow the front motors.
        leftRearConfig.follow(leftFront);
        rightRearConfig.follow(rightFront);
        
        // Invert the left side so both side drive forward with positive motor outputs
        leftFrontConfig.inverted(false);
        rightFrontConfig.inverted(true);

        leftFront.configure(leftFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftRear.configure(leftRearConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFront.configure(rightFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightRear.configure(rightRearConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Put the front motors into the differential drive object. This will control all 4 motors with
        // the rears set to follow the fronts
        m_drivetrain = new DifferentialDrive(leftFront, rightFront);
        
        // m_odometry =
        // new DifferentialDriveOdometry(
        //     m_gyro.getRotation2d(), driveLeftEncoder.getDistance(), driveRightEncoder.getDistance());
        
        m_drivetrain.setMaxOutput(DriveConstants.defaultSpeed);
    }
    
    boolean isStopped = false;
    
    /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
    * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
    public void driveArcade(double speed, double rotation) {
        m_drivetrain.arcadeDrive(speed, rotation);
    }
    
    /** 
    * Drives the robot with the given linear velocity and angular velocity
    */
    // public void drive(double speed, double rotation) {
        //     var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0.0, rotation));
        //     setSpeeds(wheelSpeeds);
        // }
        
        
        @Override
        public void periodic() {
            
            // Get the rotation of the robot from the gyro.
                        
            // Update the pose
            // m_pose = m_odometry.update(gyroAngle,
            // driveLeftEncoder.getDistance(),
            // driveRightEncoder.getDistance());
            // SmartDashboard.putNumber("Gyro ", this.getHeading());
            
        }
        
        
        public double getHeading() {
            return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
        }
        
        public void zeroHeading() {
            
            m_gyro.reset();
            
        }
        
        public void setMaxOutput(double maxOutput) {
            
            m_drivetrain.setMaxOutput(maxOutput);
            
        }
        
        public double getTurnRate() {
            
            return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
            
        }
        
    }