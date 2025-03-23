// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants; 

public class Elevator extends SubsystemBase {
    public SparkMax rightMotor;
    public SparkMax leftMotor;
    public PIDController pidController;
    
    public Elevator() {
        
        this.pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        
        // Sparkmax configs
        leftMotor = new SparkMax(ElevatorConstants.LEFT_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.RIGHT_CAN_ID, MotorType.kBrushless);
        
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.smartCurrentLimit(ElevatorConstants.SMART_CURRENT_LIMIT);
        leftMotor.configure(leftConfig, null, null);
        
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.follow(leftMotor, true);
        rightConfig.smartCurrentLimit(ElevatorConstants.SMART_CURRENT_LIMIT);
        rightMotor.configure(rightConfig, null, null);

        resetEncoders();
        this.pidController.setTolerance(0.05);

    }

    public void resetEncoders() {
        leftMotor.getEncoder().setPosition(0);
    }
    
    public void run(double position) {
        // if (leftMotor.getEncoder().getPosition() > position) {
        //     while (leftMotor.getEncoder().getPosition() != position) {
        //         leftMotor.set(ElevatorConstants.ELEVATOR_LMOTOR_SPEED_DOWN);
        //         rightMotor.set(ElevatorConstants.ELEVATOR_LMOTOR_SPEED_DOWN);
        //     }
        // } else if (leftMotor.getEncoder().getPosition() < position) {
        //     while (leftMotor.getEncoder().getPosition() != position) {
        //         leftMotor.set(ElevatorConstants.ELEVATOR_LMOTOR_SPEED_UP);
        //         rightMotor.set(ElevatorConstants.ELEVATOR_LMOTOR_SPEED_UP);
        //     }
        // } 
        // this.stop();

        // if (Math.abs(leftMotor.getEncoder().getPosition()) < position) {
        //     while (Math.abs(leftMotor.getEncoder().getPosition()) < position) {
        //         runUp();
        //     }
        // } else if (Math.abs(leftMotor.getEncoder().getPosition()) > position) {
        //     while (Math.abs(leftMotor.getEncoder().getPosition()) > position) {
        //         runDown();
        //     }
        // } else if (Math.abs(leftMotor.getEncoder().getPosition()) == position) {
        //     stop();
        // }
        // stop();

        leftMotor.set(0.1*pidController.calculate(leftMotor.getEncoder().getPosition(), position));
    }

    @Override
    public void periodic() {
        // System.out.println("Current" + leftMotor.getOutputCurrent());
        // System.out.println("Encoder" + leftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator Encoder", Math.abs(leftMotor.getEncoder().getPosition()));
        SmartDashboard.putNumber("Elevator Error: ", pidController.getError());    
        SmartDashboard.putNumber("Elevator Setpoint: ", pidController.getSetpoint());


    }
    
    public void runUp() {
            // System.out.println("Running motors...");
            if (leftMotor.getEncoder().getPosition() >= ElevatorConstants.MAX_HEIGHT) {
                this.stop();
            } else{
            leftMotor.set(ElevatorConstants.ELEVATOR_LMOTOR_SPEED_UP);
        }   
    }
    
    public void runDown() {
        leftMotor.set(ElevatorConstants.ELEVATOR_LMOTOR_SPEED_DOWN);    

        // if (rightMotor.getOutputCurrent() > 30) {
        //     this.resetEncoders();
        // }
    }
    
    public void stop() {
        
        //leftMotor.set(ElevatorConstants.ELEVATOR_STOP);
        leftMotor.set(0.0);

    }   

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }
}