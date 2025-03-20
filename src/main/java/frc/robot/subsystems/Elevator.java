// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants; 

public class Elevator extends SubsystemBase {
    public SparkMax rightMotor;
    public SparkMax leftMotor;
    
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

        resetEncoders();
    }

    public void resetEncoders() {
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
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

        if (Math.abs(leftMotor.getEncoder().getPosition()) < position) {
            while (Math.abs(leftMotor.getEncoder().getPosition()) < position) {
                runUp();
            }
        } else if (Math.abs(leftMotor.getEncoder().getPosition()) > position) {
            while (Math.abs(leftMotor.getEncoder().getPosition()) > position) {
                runDown();
            }
        } else if (Math.abs(le  ftMotor.getEncoder().getPosition()) == position) {
            stop();
        }
        stop();
    }

    @Override
    public void periodic() {
        // System.out.println("Current" + leftMotor.getOutputCurrent());
        // System.out.println("Encoder" + leftMotor.getEncoder().getPosition());

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

        // if (rightMotor.getOutputCurrent() > 30) {
        //     this.resetEncoders();
        // }
    }
    
    public void stop() {
        rightMotor.set(0.1);
        leftMotor.set(0.1);
    }   
}