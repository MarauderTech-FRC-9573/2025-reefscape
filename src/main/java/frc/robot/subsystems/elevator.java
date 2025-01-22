// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.pioneersLib.faultCheck.DeviceTypes;
import frc.robot.pioneersLib.faultCheck.FaultManager;
import frc.robot.pioneersLib.faultCheck.FaultManager.Controller;

import static frc.robot.Constants.Elevator.*;

public class ElevatorIOSparkMax implements ElevatorIO {
    private ProfiledPIDController pidController;
    private ElevatorFeedforward ffController;

    private CANSparkMax rightMotor;
    private CANSparkMax leftMotor;
    private RelativeEncoder encoder;

    private DigitalInput limitSwitch;

    private boolean rightMotorZeroed;
    private boolean leftMotorZeroed;
    private boolean logPID;

    private double metersPerRotation;

    public ElevatorIOSparkMax() {
        // Sparkmax configs
        // TODO: Set to correct motor IDs
        FaultManager faultManager = FaultManager.getInstance();

        leftMotor = new CANSparkMax(LEFT_CAN_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(RIGHT_CAN_ID, MotorType.kBrushless);
        encoder = rightMotor.getEncoder();
        encoder.setPositionConversionFactor(1);
        encoder.setVelocityConversionFactor(1);

        rightMotor.setInverted(RIGHT_INVERTED);
        rightMotor.setIdleMode(IDLE_MODE);

        leftMotor.follow(rightMotor);
        leftMotor.setInverted(LEFT_INVERTED);
        leftMotor.setIdleMode(IDLE_MODE);

        leftMotor.setSmartCurrentLimit((int) SMART_CURRENT_LIMIT.magnitude());
        rightMotor.setSmartCurrentLimit((int) SMART_CURRENT_LIMIT.magnitude());

        rightMotor.burnFlash();
        leftMotor.burnFlash();

        // Configure FF and PID controllers, kA can be ignored for FF, PID is just PID but with a motion profile
        ffController = new ElevatorFeedforward(ELEVATOR_FF.kS, ELEVATOR_FF.kG, ELEVATOR_FF.kV, ELEVATOR_FF.kA);

        pidController = new ProfiledPIDController(0, 0, 0, null);
        pidController.setTolerance(DISTANCE_TOLERANCE.magnitude(),
        VELOCITY_TOLERANCE.magnitude());
        pidController = new ProfiledPIDController(ELEVATOR_PID.kP, ELEVATOR_PID.kI, ELEVATOR_PID.kD,
        new TrapezoidProfile.Constraints(ELEVATOR_MAX_VELOCITY_MPS,
        ELEVATOR_MAX_ACCEL_MPSSQ));
        pidController.setIZone(ELEVATOR_PID.iZone);

        metersPerRotation = DISTANCE_PER_ROTATION.magnitude();

        limitSwitch = new DigitalInput(LIMIT_SWITCH_DIO);

        // No null pointers
        rightMotorZeroed = false;
        leftMotorZeroed = false;
        logPID = true;

        faultManager.addDevice(faultManager.new Controller("Elevator Right Motor", DeviceTypes.SPARK, rightMotor));
        faultManager.addDevice(faultManager.new Controller("Elevator Left Motor", DeviceTypes.SPARK, leftMotor));
    }
    @Override
    public void updateOutputs(ElevatorIOOutputs outputs) {
        outputs.leftMotorCurrent = leftMotor.getAppliedOutput();
        outputs.rightMotorCurrent = rightMotor.getAppliedOutput();
    }

    // Sets the end state of the PID controller, don't need to do anything for ff because it bases its setpoint off the PID setpoint
    @Override
    public void setGoal(State goalState) {
        if (!goalState.equals(pidController.getGoal())) pidController.setGoal(goalState);
    }

    // Basically a periodic, runs the motor velocity based on the goal state using FF + profile PID
    // Accounting for gear ratio in getPosition is unecessary because meters per rotation takes it into account
    @Override
    public void runDistance() {
        rightMotor.setVoltage(pidController.calculate(encoder.getPosition() * metersPerRotation)
                + ffController.calculate(pidController.getSetpoint().velocity));

    }

    @Override
    public void setLogPID(boolean logPID) {
        this.logPID = logPID;
    }

    // TODO: Does left motor volts also need to be set
    // Open loop control for SYSID
    @Override
    public void runVolts(Measure<Voltage> volts) {
        rightMotor.setVoltage(volts.magnitude());
    }

    // At setpoint for state transitions
    @Override
    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    public void zero() {
        double leftZeroingSpeed = -ZEROING_SPEED;
        double rightZeroingSpeed = -ZEROING_SPEED;

        if (rightMotor.getOutputCurrent() > ZEROING_CURRENT_LIMIT.magnitude() || !limitSwitch.get()) {
            rightZeroingSpeed = 0;
            if (!rightMotorZeroed) encoder.setPosition(0); rightMotorZeroed = true;
        }

        if (leftMotor.getOutputCurrent() > ZEROING_CURRENT_LIMIT.magnitude() || !limitSwitch.get()) {
            leftZeroingSpeed = 0;
            if (!leftMotorZeroed) encoder.setPosition(0); leftMotorZeroed = true;
        }

        rightMotor.set(rightZeroingSpeed);
        leftMotor.set(leftZeroingSpeed);
    }

    @Override
    public boolean elevatorZeroed() {
        return leftMotorZeroed && rightMotorZeroed;
    }
    
}