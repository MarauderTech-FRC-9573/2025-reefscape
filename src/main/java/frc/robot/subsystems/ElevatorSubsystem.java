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

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax rightMotor;
    private final SparkMax leftMotor;
    private final PIDController pidController;

    public ElevatorSubsystem() {
        this.pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

        // Initialize motors
        leftMotor = new SparkMax(ElevatorConstants.LEFT_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.RIGHT_CAN_ID, MotorType.kBrushless);

        // Configure motors
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.smartCurrentLimit(ElevatorConstants.SMART_CURRENT_LIMIT);
        leftMotor.configure(leftConfig, null, null);

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.follow(leftMotor, true);
        rightConfig.smartCurrentLimit(ElevatorConstants.SMART_CURRENT_LIMIT);
        rightMotor.configure(rightConfig, null, null);

        resetEncoders();
        pidController.setTolerance(0.05);
    }

    public void resetEncoders() {
        leftMotor.getEncoder().setPosition(0);
    }

    public void manualControl(double speed) {
        leftMotor.set(speed);
    }

    public void moveToSetpoint(double setpoint) {
        double output = pidController.calculate(leftMotor.getEncoder().getPosition(), setpoint);
        leftMotor.set(output);
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", leftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator Setpoint", pidController.getSetpoint());
        SmartDashboard.putBoolean("At Setpoint", atSetpoint());
    }
}