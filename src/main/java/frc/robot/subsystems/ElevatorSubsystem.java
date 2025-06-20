// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax rightMotor;
    private final SparkMax leftMotor;
    private final PIDController pidController;
    private PIDController glassEditPIDController;
    private static final double GRAVITY_FEEDFORWARD = 0.05; // Tune as needed

    private double targetPosition = 0.0;
    private boolean manualOverride = false;
    private double manualSpeed = 0.0;

    public ElevatorSubsystem() {
        this.pidController = new PIDController(0,0,0);
        SmartDashboard.putNumber("P", 0);
        SmartDashboard.putNumber("I", 0);
        SmartDashboard.putNumber("D", 0);

        this.glassEditPIDController = new PIDController(0,0 ,0 );

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
        pidController.setTolerance(0.1);
        targetPosition = getCurrentPosition();
    }

    public void resetEncoders() {
        leftMotor.getEncoder().setPosition(0);
        targetPosition = 0.0;
    }

    public double getCurrentPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    // Set the target position for the elevator to hold or move to
    public void setTargetPosition(double position) {
        targetPosition = position;
        manualOverride = false;
    }

    // Used by manual control command
    public void manualControl(double speed) {
        manualOverride = true;
        // Default Speed of 0.1
        manualSpeed = speed == 0.0 ? 0.2 : MathUtil.clamp(speed, -ElevatorConstants.ELEVATOR_MOTORS_MAX_SPEED, ElevatorConstants.ELEVATOR_MOTORS_MAX_SPEED);
        // Optionally, update targetPosition to current so it holds here after manual
        targetPosition = getCurrentPosition();
    }

    // Called every loop to move to the target position (unless in manual override)
    private void moveToSetpoint(double setpoint) {
        double output = pidController.calculate(getCurrentPosition(), setpoint);
        output = MathUtil.clamp(output, -ElevatorConstants.ELEVATOR_MOTORS_MAX_SPEED, ElevatorConstants.ELEVATOR_MOTORS_MAX_SPEED);

        if (atSetpoint()) {
            leftMotor.set(ElevatorConstants.ELEVATOR_STOP); // Stop motors if at setpoint
        }
        leftMotor.set(output + GRAVITY_FEEDFORWARD);
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getCurrentPosition());
        SmartDashboard.putNumber("Elevator Setpoint", targetPosition);
        SmartDashboard.putBoolean("Elevator At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Elevator Speed: ", manualSpeed);
        SmartDashboard.putNumber("Elevator Current: ", Math.abs(leftMotor.getOutputCurrent()));
        SmartDashboard.putBoolean("Manual Override Status: ", manualOverride);
        glassEditPIDController = new PIDController(SmartDashboard.getNumber("P", 0 ), SmartDashboard.getNumber("I", 0),SmartDashboard.getNumber("I", 0));

        if (manualOverride) {
            leftMotor.set(manualSpeed);
            // Optionally, update targetPosition to current so it holds here after manual
            targetPosition = getCurrentPosition();
            // Resets encoders if it detects we've hit the bottom
            // Value based on current draw when forcing elevator to bottom
            if (leftMotor.getOutputCurrent() > 35) {
                this.resetEncoders();
            }
        } else {
            moveToSetpoint(targetPosition);
        }
    }

    // Call this when manual control ends
    public void endManualControl() {
        manualOverride = false;
        // targetPosition is already set to current position in manualControl
    }
}