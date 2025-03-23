package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Pivot extends SubsystemBase {
    private SparkMax pivot;
    private SparkLimitSwitch beamBreaker;
    private PIDController pidController;
    public double targetPos;

    
    public Pivot() {
        pivot = new SparkMax(16, MotorType.kBrushless);
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.smartCurrentLimit(PivotConstants.SMART_CURRENT_LIMIT);
        pivot.configure(pivotConfig, null, null);
        resetEncoders();

        this.beamBreaker = pivot.getForwardLimitSwitch();

        this.pidController = new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);
        this.pidController.setTolerance(0.05);
    }

    public void resetEncoders() {
        pivot.getEncoder().setPosition(0);
    }

    public SparkLimitSwitch getBeamBreak() { 
        return this.beamBreaker;
    }

    public void run(double position) {
        // if (pivot.getEncoder().getPosition() < position) {
        //     while (pivot.getEncoder().getPosition() < position) {
        //         runUp();
        //     }
        // } else if (pivot.getEncoder().getPosition() > position) {
        //     while (pivot.getEncoder().getPosition() > position) {
        //         runDown();
        //     }
        // } else if (pivot.getEncoder().getPosition() == position) {
        //     stop();
        // }
        // stop();

        pivot.set(0.1*pidController.calculate(pivot.getEncoder().getPosition(), position));
        
    }
    
    public void runUp() {

        // if (pivot.getOutputCurrent() > 30) {
        //     this.resetEncoders();
        // }
        //if (pivot.getEncoder().getPosition() <= PivotConstants.MAX_RETRACTION) {
            pivot.set(PivotConstants.PIVOT_SPEED_UP);
        //} else {
        //    stop();
        //}
    }
    public void runDown() {

            pivot.set(PivotConstants.PIVOT_SPEED_DOWN);
    }

    @Override
    public void periodic() {
        // System.out.println("Current" + pivot.getOutputCurrent());
        // System.out.println("Encoder" + pivot.getEncoder().getPosition());
        SmartDashboard.putNumber("Pivot Encoder", pivot.getEncoder().getPosition());
        SmartDashboard.putBoolean("Pivot Beambreak", beamBreaker.isPressed());
        SmartDashboard.putNumber("Pivot Error: ", pidController.getError());    
        SmartDashboard.putNumber("Pivot Setpoint: ", pidController.getSetpoint());

    }
    
    // If statement checks if pivot is in upright position
    // or not before engaging the motors
    // Upright Bpos is closer to elevator, Upright Tpos is farther from elevator
    // (all Encoder values are negative)
    public void stop() {
        // if (pivot.getEncoder().getPosition() >= -9.0) { 
        //     // System.out.println("Running Bspeed");
        //     pivot.set(PivotConstants.PIVOT_STOP_BSPEED); // Will crash into funnel we cant fix rn lol
        // } else {
        //     pivot.set(PivotConstants.PIVOT_STOP_FSPEED);
        
        // }

        pivot.set(0);
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }
}
