// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static int kOperatorControllerPort = 1;
  }
  
  public final class PivotConstants {
    // TODO: Replace with actual CAN ID of the pivot motor
    
    public static final int PIVOT_MOTOR_ID = 16;
    public static final int SMART_CURRENT_LIMIT = 40;
    // TODO: Test the pivot motor with these speeds and determine ideal values
    
    public static final double PIVOT_SPEED_UP = 0.15;
    public static final double PIVOT_SPEED_DOWN = -0.15;
    public static final double MAX_EXTENSION = -15;
    public static final double MAX_RETRACTION = -7.5;
    public static final double PIVOT_L4 = 0.0;
    public static final double PIVOT_L3 = 0.0;
    public static final double PIVOT_L2 = 0.0;
    public static final double PIVOT_L1 = 0.0;
    public static final double CORAL_STATION_POSITION = -6.4;
    public static final double L4_POSITION = -10.738;
    public static final double L3_POSITION = -9.0;
    public static final double L2_POSITION = -9.0;
    public static final double L1_POSITION = -6.3;
    public static final double PIVOT_UPRIGHT_BPOSITION = -8.5;
    public static final double PIVOT_UPRIGHT_TPOSITION = -9.5;
    public static final double PIVOT_STOP_BSPEED = -0.007;
    public static final double NET_POSITION = -1.833;
    public static final double PIVOT_STOP_FSPEED = 0.0225;

    public static final double PIVOT_NOCLIP = -8;



    public static final double kP = 0.7;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
    
    
    
    
    
  }
  
  public static class ManipulatorConstants {
    // TODO: Replace with actual CAN ID of the manipulator motor
    public static final int MANIPULATOR_MOTOR_ID = 17;
    // TODO: Test the end effector with these speeds and determine ideal values
    public static final double CORAL_INTAKE_SPEED = 0.1;
    public static final double CORAL_SCORE_SPEED = -0.5;
    public static final double ALGAE_INTAKE_SPEED = 0.5;
    public static final double ALGAE_SCORE_SPEED = -0.5;
    public static final double MANIPULATOR_BACKWARD_STOP = 30;
    public static final double MANIPULATOR_STOP = 0;
    
  }
  
  public static class SpeedConstants {
    public static final double speedMax = 1.5;
    public static final double speedMin = 0.5;
    public static final double speedDefault = 1;
  }
  
  public static class ElevatorConstants {
    
    public static final int LEFT_CAN_ID = 14;
    public static final int RIGHT_CAN_ID = 15;
    public static final boolean RIGHT_INVERTED = false;
    public static final String IDLE_MODE = null;
    public static final int SMART_CURRENT_LIMIT = 40;
    public static final int ELEVATOR_FF_kS = 0;
    public static final double ELEVATOR_RMOTOR_SPEED_UP = -0.20;
    public static final double ELEVATOR_LMOTOR_SPEED_UP = 0.20;
    public static final double ELEVATOR_RMOTOR_SPEED_DOWN = 0.20;
    public static final double ELEVATOR_LMOTOR_SPEED_DOWN = -0.20;
    public static final double ELEVATOR_STOP = 0.1;    
    public static final double L1_ENCODER = 11.5;
    public static final double L2_ENCODER = 23.3;
    public static final double L3_ENCODER = 45.25;
    public static final double L4_ENCODER = 76;
    public static final double RMOTOR_STOP_SPEED = -0.01;
    public static final double LMOTOR_STOP_SPEED = 0.01;
    public static final double MAX_HEIGHT = 76;
    public static final Runnable CORAL_STATION_ENCODER = null;
    public static final double LOWER_ALGAE_ENCODER = 40.6;
    public static final double UPPER_ALGAE_ENCODER = 58;
    public static final double NET_ENCODER = 75;

    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    
  }
  
  public static final double maxSpeed = Units.feetToMeters(4.5);
}
