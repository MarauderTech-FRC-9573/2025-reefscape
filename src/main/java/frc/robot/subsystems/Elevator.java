// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants; 

public class Elevator extends SubsystemBase {
    public SparkMax rightMotor;
    public SparkMax leftMotor;

    public SparkClosedLoopController m_leftController = leftMotor.getClosedLoopController();
    public SparkClosedLoopController m_rightController = rightMotor.getClosedLoopController();

    public Elevator() {
        // Sparkmax configs
        leftMotor = new SparkMax(ElevatorConstants.LEFT_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.RIGHT_CAN_ID, MotorType.kBrushless);
        
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.smartCurrentLimit(ElevatorConstants.SMART_CURRENT_LIMIT);
        leftConfig.closedLoop.p(1).i(0).d(0).outputRange(0, 75.0);
        leftMotor.configure(leftConfig, null, null);
        
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.smartCurrentLimit(ElevatorConstants.SMART_CURRENT_LIMIT);
        rightConfig.closedLoop.p(1).i(0).d(0).outputRange(0, 75.0);
        rightMotor.configure(rightConfig, null, null);

    }

    public void resetEncoders() {
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
    }


    public void runUp() {
            // System.out.println("Running motors...");
            if (leftMotor.getEncoder().getPosition() >= ElevatorConstants.MAX_HEIGHT) {
                this.stop();
            } else{
            rightMotor.set(ElevatorConstants.ELEVATOR_RMOTOR_SPEED_UP);
            leftMotor.set(ElevatorConstants.ELEVATOR_LMOTOR_SPEED_UP); 
        }   
    }
    
    public void runDown() {
        rightMotor.set(ElevatorConstants.ELEVATOR_RMOTOR_SPEED_DOWN);
        leftMotor.set(ElevatorConstants.ELEVATOR_LMOTOR_SPEED_DOWN);    

        if (rightMotor.getOutputCurrent() > 30) {
            this.resetEncoders();
        }
    }
    
    public void stop() {
        rightMotor.set(ElevatorConstants.RMOTOR_STOP_SPEED);
        leftMotor.set(ElevatorConstants.LMOTOR_STOP_SPEED);
    }   
}