// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
    public static final double DEADBAND = 0.05;
  }


  public static class SpeedConstants {
    public static final double speedMax = 1.0;
    public static final double speedMin = 0.1;
    public static final double speedDefault = 0.8; 
  }

  public static class SwerveConstants{
    public static final double WHEELBASE_METERS = Units.inchesToMeters(70);
    public static final double TRACKWIDTH_METERS = Units.inchesToMeters(70);

  }

public static final double maxSpeed = Units.feetToMeters(4.5);


public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
  // Front left
  new Translation2d(SwerveConstants.WHEELBASE_METERS / 2.0, SwerveConstants.TRACKWIDTH_METERS / 2.0), 
  // Front right
  new Translation2d(SwerveConstants.WHEELBASE_METERS / 2.0, SwerveConstants.TRACKWIDTH_METERS / 2.0), 
  // Back left
  new Translation2d(SwerveConstants.WHEELBASE_METERS / 2.0, SwerveConstants.TRACKWIDTH_METERS / 2.0), 
  // Back right
  new Translation2d(SwerveConstants.WHEELBASE_METERS / 2.0, SwerveConstants.TRACKWIDTH_METERS / 2.0)

  );
}

