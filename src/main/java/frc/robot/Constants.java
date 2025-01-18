// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants
  {
    // PWM ports/CAN IDs for motor controllers
    public static final int kLeftRearID = 5;
    public static final int kLeftFrontID = 4;
    public static final int kRightRearID = 2;
    public static final int kRightFrontID = 3;
    
    // Current limit for drivetrain motors
    public static final int kDriveCurrentLimit = 40;
    
    public static int driverControllerPort = 0;
    public static int operatorControllerPort = 1;
    
    //PID values for gyro taken from wpilib gyrocommand example
    
    public static final boolean kGyroReversed = false;
    
    public static final double kTurnP = 0;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    
    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;
    
    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
    
    // varys per robot and must be tuned 
    // try Robot Characterization Toolsuite to get these values
    // These values are not used anywhere on the robot
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kvVoltSecondsPerRadian = 1.5;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;
    
    public static double kWheelDiameterMeters = 0.15;
    
    // default speed of the robot
    public static double precisionSpeed = 0.2;
    public static double defaultSpeed = 0.8;
    public static double turboSpeed = 1.0;
  }
}
