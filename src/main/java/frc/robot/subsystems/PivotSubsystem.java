package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PivotSubsystem extends SubsystemBase {
    private SparkMax pivotMotor;
    private SparkLimitSwitch beamBreaker;
    private PIDController pidController;

    public double targetPosition = 0.0;
    private boolean manualOverride = false;
    private double manualSpeed = 0.0;

    public PivotSubsystem() {
        this.pidController = new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);

        pivotMotor = new SparkMax(PivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.smartCurrentLimit(PivotConstants.SMART_CURRENT_LIMIT);
        pivotConfig.inverted(true);
        pivotMotor.configure(pivotConfig, null, null);
        resetEncoders();

        this.beamBreaker = pivotMotor.getForwardLimitSwitch();

        this.pidController.setTolerance(0.5);
        targetPosition = getCurrentPosition();
    }
    
    public void resetEncoders() {
        pivotMotor.getEncoder().setPosition(0);
        targetPosition = 0.0;
    }

    public SparkLimitSwitch getBeamBreak() {
        return this.beamBreaker;
    }

    public double getCurrentPosition() {
        return pivotMotor.getEncoder().getPosition();
    }

    // Set the target position for the pivot to hold or move to
    public void setTargetPosition(double position) {
        targetPosition = position;
        manualOverride = false;
    }

    public void manualControl(double speed) {
        manualOverride = true;
        manualSpeed = speed == 0.0 ? 0.2
                : MathUtil.clamp(speed, -ElevatorConstants.ELEVATOR_MOTORS_MAX_SPEED,
                        ElevatorConstants.ELEVATOR_MOTORS_MAX_SPEED);
        targetPosition = getCurrentPosition();
    }
    
    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    public void moveToSetpoint(double setpoint) {
        double output = pidController.calculate(getCurrentPosition(), setpoint);
        output = MathUtil.clamp(output, -PivotConstants.PIVOT_MOTOR_MAX_SPEED, PivotConstants.PIVOT_MOTOR_MAX_SPEED);
        pivotMotor.set(output + PivotConstants.GRAVITY_FEEDFORWARD);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", getCurrentPosition());
        SmartDashboard.putBoolean("Beam Breaker", beamBreaker.isPressed());
        SmartDashboard.putNumber("Pivot Setpoint", targetPosition);
        SmartDashboard.putBoolean("Pivot At Setpoint", atSetpoint());

        if (manualOverride) {
            pivotMotor.set(manualSpeed);
            targetPosition = getCurrentPosition(); // Update target position to current if in manual mode
        } else {
            moveToSetpoint(targetPosition);
        }
    }

    public void endManualControl() {
        manualOverride = false;
    }


}