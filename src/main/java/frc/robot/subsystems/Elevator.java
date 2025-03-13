// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants; 

public class Elevator extends SubsystemBase {
    private SparkMax rightMotor;
    private SparkMax leftMotor;
    PIDController pidController = new PIDController(1, 0, 0.001);
    
    public Elevator() {
        // Sparkmax configs
        leftMotor = new SparkMax(ElevatorConstants.LEFT_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.RIGHT_CAN_ID, MotorType.kBrushless);

        
        
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.smartCurrentLimit(ElevatorConstants.SMART_CURRENT_LIMIT);
        leftMotor.configure(leftConfig, null, null);
        
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.smartCurrentLimit(ElevatorConstants.SMART_CURRENT_LIMIT);
        rightMotor.configure(rightConfig, null, null);
        
    }
    
    public void resetEncoders() {
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
    }
    
    @Override
    public void periodic() {
        System.out.println("Current" + leftMotor.getOutputCurrent());
        System.out.println("Encoder" + leftMotor.getEncoder().getPosition());
        
    }

    public void elevator() {
        leftMotor.set(pidController.calculate(leftMotor.getEncoder().getPosition(), 1));
    }
    
    public void run(double setpoint) {
        leftMotor.getClosedLoopController().setReference(setpoint, SparkBase.ControlType.kMAXMotionPositionControl);
        rightMotor.getClosedLoopController().setReference(setpoint, SparkBase.ControlType.kMAXMotionPositionControl);
    }

    public void runUp() {
        leftMotor.set(ElevatorConstants.ELEVATOR_LMOTOR_SPEED_UP);
        leftMotor.set(ElevatorConstants.ELEVATOR_RMOTOR_SPEED_UP);

    }

    public void runDown() {
        leftMotor.set(ElevatorConstants.ELEVATOR_LMOTOR_SPEED_DOWN);
        leftMotor.set(ElevatorConstants.ELEVATOR_RMOTOR_SPEED_DOWN);

    }

    
    public void stop() {
        rightMotor.set(0);
        leftMotor.set(0);
    }    
    
}
