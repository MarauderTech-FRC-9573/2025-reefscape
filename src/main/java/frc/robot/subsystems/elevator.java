// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants; 

public class Elevator extends SubsystemBase {
    private PIDController pidController;
    
    private SparkMax rightMotor;
    private SparkMax leftMotor;
    
    private double desiredTarget;
    
    
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
        
        pidController = new PIDController(1.0, 0.0, 0.0);
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


    
    public void runUp() {
            // System.out.println("Running motors...");
            rightMotor.set(ElevatorConstants.ELEVATOR_RMOTOR_SPEED_UP);
            leftMotor.set(ElevatorConstants.ELEVATOR_LMOTOR_SPEED_UP);    
    }
    
    public void runDown() {
        rightMotor.set(ElevatorConstants.ELEVATOR_RMOTOR_SPEED_DOWN);
        leftMotor.set(ElevatorConstants.ELEVATOR_LMOTOR_SPEED_DOWN);    

        if (rightMotor.getOutputCurrent() > 30) {
            this.resetEncoders();
        }
    }
    
    public void stop() {
        rightMotor.set(0);
        leftMotor.set(0);
    }
    
    public void L1() {
        if (Math.abs(leftMotor.getEncoder().getPosition()) < ElevatorConstants.L1_ENCODER) {
            while (Math.abs(leftMotor.getEncoder().getPosition()) < ElevatorConstants.L1_ENCODER) {
                this.runUp();
            }
        } else if (Math.abs(leftMotor.getEncoder().getPosition()) > ElevatorConstants.L1_ENCODER) {
            while (Math.abs(leftMotor.getEncoder().getPosition()) > ElevatorConstants.L1_ENCODER) {
                this.runDown();
            }
        }
        this.stop();
    }
    
    public void L2() {
        if (Math.abs(leftMotor.getEncoder().getPosition()) < ElevatorConstants.L2_ENCODER) {
            while (Math.abs(leftMotor.getEncoder().getPosition()) < ElevatorConstants.L2_ENCODER) {
                this.runUp();
            }
        } else if (Math.abs(leftMotor.getEncoder().getPosition()) > ElevatorConstants.L2_ENCODER) {
            while (Math.abs(leftMotor.getEncoder().getPosition()) > ElevatorConstants.L2_ENCODER) {
                this.runDown();
            }
        }
        this.stop();
        
    }
    
    public void L3() {
        if (Math.abs(leftMotor.getEncoder().getPosition()) < ElevatorConstants.L3_ENCODER) {
            while (Math.abs(leftMotor.getEncoder().getPosition()) < ElevatorConstants.L3_ENCODER) {
                this.runUp();            
            }
        } else if (Math.abs(leftMotor.getEncoder().getPosition()) > ElevatorConstants.L3_ENCODER) {
            while (Math.abs(leftMotor.getEncoder().getPosition()) > ElevatorConstants.L3_ENCODER) {
                this.runDown();
            }
        }
        this.stop();
    }
    
    
    public void L4() {
        if (Math.abs(leftMotor.getEncoder().getPosition()) > ElevatorConstants.L4_ENCODER) {
            while (Math.abs(leftMotor.getEncoder().getPosition()) > ElevatorConstants.L4_ENCODER) {
                this.runDown();
            }
        } else if (Math.abs(leftMotor.getEncoder().getPosition()) < ElevatorConstants.L4_ENCODER) {
            while (Math.abs(leftMotor.getEncoder().getPosition()) < ElevatorConstants.L4_ENCODER) {
                this.runUp();
            }
        }
        this.stop();
        
    }
    
    
    
}
