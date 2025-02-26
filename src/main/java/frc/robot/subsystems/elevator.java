// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants; 

public class Elevator extends SubsystemBase {
    private ProfiledPIDController pidController;
    
    private SparkMax rightMotor;
    private SparkMax leftMotor;
    
    private double desiredTarget;
    
    public enum ElevatorState { 
        // TODO: set encoder positions for each level of the reef
        L1(10), 
        L2(20), 
        L3(30),
        L4(40),
        NET(50);
        final double encoderPosition;
        
        ElevatorState(double encoderPosition) {
            this.encoderPosition = encoderPosition;
        }
        
        public double getEncoderPosition() {
            return this.encoderPosition;
        }
    }
    
    public Elevator() {
        // Sparkmax configs
        leftMotor = new SparkMax(ElevatorConstants.LEFT_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.RIGHT_CAN_ID, MotorType.kBrushless);
        rightMotor.getEncoder();
        
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.follow(ElevatorConstants.RIGHT_CAN_ID);
        leftConfig.smartCurrentLimit(ElevatorConstants.SMART_CURRENT_LIMIT);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.smartCurrentLimit(ElevatorConstants.SMART_CURRENT_LIMIT);
        
        // Would help prevent brownouts if needed
        // rightMotor.burnFlash();
        // leftMotor.burnFlash();
        
        // Configure FF and PID controllers, kA can be ignored for FF, PID is just PID but with a motion profile
        pidController = new ProfiledPIDController(0, 0, 0, null);
        pidController.setTolerance(ElevatorConstants.DISTANCE_TOLERANCE,
        ElevatorConstants.VELOCITY_TOLERANCE);
        pidController = new ProfiledPIDController(ElevatorConstants.ELEVATOR_PID_kP, ElevatorConstants.ELEVATOR_PID_kI, ElevatorConstants.ELEVATOR_PID_kD,
        new TrapezoidProfile.Constraints(ElevatorConstants.ELEVATOR_MAX_VELOCITY_MPS,
        ElevatorConstants.ELEVATOR_MAX_ACCEL_MPSSQ));
        pidController.setIZone(ElevatorConstants.ELEVATOR_PID_iZone);
    }
    
    public void run() {
        leftMotor.set(desiredTarget);
    }
    
    public void setDesiredTarget(ElevatorState desiredState) {
        this.desiredTarget = desiredState.getEncoderPosition();
    }
    
    public boolean isStalled() {
        // TODO: test to see what actual current draw is when the elevator is at the bottom
        if (leftMotor.getOutputCurrent() > 100) {
            return true;
        } else {
            return false;
        }
    }
}
